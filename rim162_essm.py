from serverimports import *
import sys, linecache, base, wic
import wic.common.math as math
import math as pdmath
import wicg
import wicp
import predictorFCS_mxre_includes__ as mx
from predictorFCS_flint__includes__ import *
from predictorFCS_CAI__includes__ import *
from wic.common import StringToInt as s2i
from exwcc.common import *
from exwcc.awsd import *
from exwcc.ecpd import *
from exflint.common import *
from exflint.wico import *

"""
	RIM-162 ESSM wico_driver script for EX-WCC 10.0
	Version 1.0
	June 2, 2020
	(c) www.wicmwmod.com, Massgate.org
	
	NB:  Future SM-6 and SM-3 development will occur on this same script.
	NB(2):  wic.game.ShooterBase drivers for FLINT must always return False on Aim().
	The game will loop Aim() at every frame when returning False.
"""

class FLINT_Aegis_Drivers( exflint.wico.ShooterBase ):
	def __init__( self, a = None, b = None ):
		if wicg.GetUnitMember( self.Host.Id, 'type' ) == s2i('FLINT_AAM_ESSM'):
			self.m_type__msl = 9
		else:
			self.m_type__msl = 6
		
		MCO_LOAD_FLINT(self.Host.Id, self.m_type__msl)
		
		self._vls_init = False
		self.PN_Start_Time = 0.9
		self._isp_max = 132.0
		self._isp = 59.4
		self.Constant_Drag_Factor = 0.025
		self.Max_Speed = 270.0
		self.Propellant_BurnTime = 5.5
	
	def Aim( self ):
		if self.feedback_update_time is not 0 and ( ( wic.common.GetCurrentTime() - self.feedback_update_time ) < EXFLINT_TICKTOCK ):
			return False
		
		self.feedback_update_time = wic.common.GetCurrentTime()
		
		def ex_socket():
			global ecl, MC_MDC_INS
			
			if self.initp == 0:
				self.Host.SetInvulnerable( True )
				UnTrackUnit( self.Host.Id )
				
				self.current_state = math.Vector3( self.Host.Position )
				self.IMU_Missile_Start_Position = math.Vector3( self.Host.Position )
				
				# Initialize the Missile Guidance System
				try:
					msl_to_track = MC_MDC_INS[self.Host.Id]
				except:
					pass
				
				# Download target directional vector from launch host
				self.PN_Terminal_Basket = self.GetTargetPosition()
				
				# With Aegis, we now always set midcourse to 1.  When missile needs to go LOBL, we will
				# exit out of mdc in the homing loop function after seeker has initialized.
				self.PN_INS_MidCourse = 1
				
				# Mk41 VLS vertical launch orientation
				launch_rail = ( self.GetTargetPosition() - self.Host.Position ).NormalizeSafe()
				launch_rail.Y = 100.0
				
				self.Total_Velocity_Vector = launch_rail.NormalizeSafe()
				self.MissileStartTime = self.i_time
				
				self.VLS_Init_Turn = math.Vector3( self.Host.Position )
				self.VLS_Init_Turn += ( self.GetTargetPosition() - self.Host.Position ).NormalizeSafe() * 400.0
				
				self.prev_range = 0
				
				# EXFLINT 651 - weight impulse calculation
				t_frames = self.Propellant_BurnTime / EXFLINT_TICKTOCK
				self._isp_diff = ( self._isp_max - self._isp ) / t_frames
			
			return True
		
		def ex_propulsion():
			global MBI_ENV_EXFLINT_DBG
			
			impulse = self._isp
			if not self.vMax_Reached and ( self.i_time - self.MissileStartTime ) > self.Propellant_BurnTime:
				self.vMax_Reached = 1
			
			if MBI_ENV_EXFLINT_DBG:
				# EXFLINT70
				try:
					wic.game.Players[1].ChatMessage("<#ff0>EX TELEMETRY:</>  Vcur: %s  Altitude: %s km  Mission Time: T+%s" % ( round( self.IMU_Measured_VR, 1 ), round( self.IMU_Current_Alt/100, 2 ), round( self.i_time, 3 ) ), 1 )
					
					if self.vMax_Reached is 0:
						ve = round(impulse,1)
					else:
						ve = "<#ccc>No Thrust</>"
						
					try:
						self.__imu_dv0
						
						dV = abs(self.IMU_Measured_VR - self.__imu_dv0)/EXFLINT_TICKTOCK
						g_force = EXFLINT_G_Force( self.IMU_Measured_VR, self.__imu_dv0, self.vMax_Reached )
						
						self.__imu_dv0 = self.IMU_Measured_VR
						
					except AttributeError, NameError:
						self.__imu_dv0 = self.IMU_Measured_VR
						dV = 0.0
					
					alt_si_meters = ( self.IMU_Current_Alt / 100.0 ) * 1000.0
					Pa = 1.01325 * pow( 1 - (2.25577 * 0.00001 * alt_si_meters), 5.25588 )
					
					vv = self.IMU_Measured_VR * EXFLINT_TICKTOCK
					q = 0.5 * Pa * vv**2
					
					wic.game.Players[1].ChatMessage("<#ff0>EX TELEMETRY:</>  IAS: Mach %s  G Load: %s  AoA: %s  Control Input: %s" % ( round( self.IMU_Measured_VR/60.0, 1 ), round( g_force, 1 ), round( math.Vector3( self.PN_Required_Direction_Add ).Y, 1 ), round( self.PN_Required_Direction_Add.Length(), 1 ) ), 1 )
					wic.game.Players[1].ChatMessage("<#ff0>EX TELEMETRY:</>  Dyn Q: %s  Delta-V: %s wm/s  TAS: %s km/h  Impulse: %s" % ( round( q, 2 ), round( dV, 2 ), round((self.IMU_Measured_VR/60.0) * 0.34 * 60 * 60, 1), ve ), 1)
				except:
					wic.game.Players[1].ChatMessage("<#ff0>EX TELEMETRY:</>  Awaiting Vehicle Downlink..", 1 )
			
			if EXFLINT_Integrate( self.current_state, self.vMax_Reached, self.Total_Velocity_Vector, self.PN_Required_Direction_Add, self.PN_Update_Ticket, self.IMU_Measured_VR, self.Max_Speed, self.IMU_Current_Alt, self.Constant_Drag_Factor, impulse, 0, True ) is 1:
				self.PN_Update_Ticket = 0
				self.Prop_Steering_Tic = 1
			
			# Required
			self.initp = 1
			
			# EXFLINT 651 - weight calculation
			self._isp = min( self._isp_max, self._isp + self._isp_diff )
			
			return True
		
		def aegis_maintain_terminal_lock():
			global illuminator, MC_SARH, MC_AEGIS_MCU
			
			if self.tac_target < 0 or self.IMU_Measured_VR < 30:
				# invalid track or insufficient energy to command any meaningful acceleration
				return False
			
			try:
				mc_key = MC_SARH[wic.game.Units[self.tac_target].Id]
				ship = MC_AEGIS_MCU[mc_key]
				current_pp = ecl[mc_key].ReturnPPNS()
			except:
				return False
			
			# performance optimization
			if self.IRT_EO_Time is not 0 and ( self.i_time - self.IRT_EO_Time ) < 0.1:
				return True
			
			track_verified = False
			if current_pp != 2 and current_pp < 4:
				try:
					if self.tac_target in wic.game.GetVisibleEnemyUnitsInArea( self.Host.Team, wic.game.Units[self.tac_target].Position, 5 ):
						track_verified = True
				except:
					return False
			else:
				track_verified = True
			
			# Check for illumination, boresight constraints and doppler effects
			if track_verified and illuminator[ship].waveform(mc_key):
				try:
					toTarget = math.Vector3( wic.game.Units[self.tac_target].Position ) - self.current_state
					t_alt = math.Vector3(wic.game.Units[self.tac_target].Position).Y - wic.common.GetY( wic.game.Units[self.tac_target].Position.X, wic.game.Units[self.tac_target].Position.Z, 1, 1 )
				except:
					# wic.game.UnknownUnitException
					return False
				
				if self.miss_enlarging > 10:
					# interceptor is off the track
					self.PATRIOT_MC_ENR_Decouple()
				
				# Seeker boresight limits
				test = -math.asin( toTarget.Y / toTarget.Length() )
				if ( math.AngleDiff( math.atan2( toTarget.X, toTarget.Z ), self.Host.BodyHeading ) > self.IRT_Seeker_Boresight or
					math.AngleDiff( test, self.CurrentAimingPitch ) > self.IRT_Seeker_Boresight ):
					return False
				
				if self.PN_Target_Direction_Vector:
					target_dir = math.atan2( self.PN_Target_Direction_Vector.X, self.PN_Target_Direction_Vector.Z )
					my_dir = math.atan2( self.Total_Velocity_Vector.X, self.Total_Velocity_Vector.Z )
					angular_diff = math.AngleDiff( target_dir, my_dir )
				else:
					angular_diff = 0.0
				
				if 1.04 < angular_diff < 2.09 and t_alt < 30 and 1 < wic.game.Units[self.tac_target].GetCurrentSpeed() < 35:
					# LDSD - target has notched us
					return False
				elif t_alt < 5 and wic.common.Random() < 0.9:
					return False
				
				self.IRT_EO_Time = self.i_time
				return True
			
			# wic.game.Players[1].ChatMessage("fndebug: 10796: track_v = %s  wave = %s" % ( track_verified, illuminator[ship].waveform(mc_key) ) )
			return False
		
		def aegis_terminal_datalink():
			global ecl, MC_MDC_INS, illuminator, MC_AEGIS_MCU, MC_SARH
			
			try:
				# try the msl data link first
				msl_to_track = MC_MDC_INS[self.Host.Id]
				mdl_heartbeat = ( wic.common.GetCurrentTime() - ecl[msl_to_track].Return_TWS_Update() )
				
				if mdl_heartbeat >= 1.0:
					return False
				
				if msl_to_track in MC_AEGIS_MCU and msl_to_track in MC_SARH:
					ship = MC_AEGIS_MCU[msl_to_track]
					if illuminator[ship].waveform(msl_to_track):
						# target is currently illuminated by aegis network
						self.tac_target = msl_to_track
						ecl[msl_to_track].Set_Terminal_Acq()
						
						if ecl[msl_to_track].ReturnPPNS() < 2:
							self.warhead_air_mode = 1
						
						return True
					else:
						# target is in data-link, but not yet illuminated.  request it now
						if illuminator[ship].illuminate(msl_to_track):
							STT_Lock(msl_to_track)
							return True
				
			except:
				# KeyError, wic.game.UnknownUnitException
				pass
			
			return False
		
		def aegis_terminal_select():
			global ecl, MC_SARH, wico_FLINT_patriot_flint_list, MC_AEGIS_MCU
			global illuminator, MC_MDC_INS
			
			TR = wic.game.GetVisibleEnemyUnitsInArea ( self.Host.Team, self.current_state, self.MDC_INS_Mid_Course_Range )
			
			t_id = -1
			for si in TR:
				# valid target?
				if wic.game.Units[si].MetaType > 0:
					continue
				
				if ( wic.game.Units[si].MaxSpeed < 0 and
					wicg.GetUnitMember( wic.game.Units[si].Id, 'type' ) not in wico_FLINT_patriot_flint_list ):
					continue
				
				AimDirectionVector = math.Vector3( wic.game.Units[si].Position ) - self.current_state
				z_len = AimDirectionVector.Length()
				# seeker aperture/power distance for detecting non-datalinked targets.
				if z_len > self.MDC_INS_Mid_Course_Minimum_Decision_Range:
					continue
				
				xz_lim = math.AngleDiff( math.atan2( AimDirectionVector.X, AimDirectionVector.Z ), self.Host.BodyHeading )
				y_lim = -math.asin( AimDirectionVector.Y / z_len )
				
				if math.AngleDiff( y_lim, self.CurrentAimingPitch ) > self.IRT_Seeker_Boresight or xz_lim > self.IRT_Seeker_Boresight:
					continue
				
				# Check with AEGIS data link
				if si in MC_AEGIS_MCU and si in MC_SARH:
					ship = MC_AEGIS_MCU[si]
					if illuminator[ship].waveform(si):
						# target is currently illuminated by aegis network
						self.tac_target = si
						ecl[si].Set_Terminal_Acq()
						
						if ecl[si].ReturnPPNS() < 2:
							self.warhead_air_mode = 1
						
						return True
					else:
						# target is in data-link, but not yet illuminated.  request it now
						if illuminator[ship].illuminate(si):
							MC_MDC_INS[self.Host.Id] = si
							STT_Lock(si)
							return True
				else:
					t_id = si
			
			if t_id > -1:
				# Aegis network illumination mode:
				# This SM-2/ESSM must have been either fired by a non-Aegis ship, or mid-course track was terminated by SPY-1 earlier.
				# Let's request one of the ships to start illuminating while we're still flying.
				for i in illuminator:
					if illuminator[i].allocate(t_id):
						# Break now, illumination beam will be found on next frame
						break
			
			return False
		
		def ex_assert():
			if not self.initp:
				return True
			
			# FLINT41
			if ( not 0 < self.current_state.X < 1500 ) or ( not 0 < self.current_state.Z < 1500 ):
				WICGFLINT_bounds_chk_fail("AEGIS/SM-2", wic.common.GetCurrentTime(), self.Host.Position)
				return False
			
			if ( wic.common.GetCurrentTime() - self.IMU_ASSERT_Time ) > 0.1:
				self.IMU_ASSERT_Time = wic.common.GetCurrentTime()
				if self.IMU_Current_Alt < 0.2 or self.IMU_Current_Alt > self.ASSERT_Max_Alt:
					return False
			
			# EO Time
			if self.IRT_EO_Time > 0:
				eo_time_len = ( self.i_time - self.IRT_EO_Time )
				
				if eo_time_len > 2 * self.IRT_EO_Timeout:
					return False
				elif eo_time_len > 0.3 * self.IRT_EO_Timeout and not self.PN_Exemption_EO_Limit:
					# We're lost, send another.
					self.PATRIOT_MC_ENR_Decouple()
					aegis_terminal_select()
			
			# LOAL to Terminal, illuminator timeout
			if self.LOAL_to_PN_Search_Time and ( self.i_time - self.LOAL_to_PN_Search_Time ) > 2.0:
				return False
			
			return True
		
		def ex_guidance():
			global ecl, illuminator, MC_AEGIS_MCU, MC_MDC_INS
			
			msl_run_time = ( self.i_time - self.MissileStartTime )
			if msl_run_time < self.PN_Start_Time:
				return True
			
			if self.PN_Exemption_EO_Limit:
				self.Prop_Steering_Tic = 1
				return True
			
			# There is no loft for ESSM.  Placeholder for SM-6
			Ballistic_Loft = False
			
			if self.PN_INS_MidCourse:
				range = ( math.Vector3(self.PN_Terminal_Basket) - self.current_state ).Length()
				
				if range <= self.MDC_INS_Mid_Course_Range and not self.PN_Terminal_Search_Mode and msl_run_time > 2:
					self.PN_Terminal_Search_Mode = 1
				
				if self.PN_Terminal_Search_Mode:
					# Terminal acquisition state:  turn on the missile seeker
					# ForceSetStance(self.Host.Id, "CROUCH")
					
					if self.LOAL_to_PN_Search_Time is 0:
						self.LOAL_to_PN_Search_Time = self.i_time
					
					if self.tac_target < 0:
						if not aegis_terminal_datalink():
							aegis_terminal_select()
						
						# we will check again next frame.
						self.Prop_Steering_Tic = 1
						return False
					
					try:
						PN_Track = wic.game.Units[ self.tac_target ].Position
					except:
						# wic.game.UnknownUnitException
						self.Prop_Steering_Tic = 1
						self.tac_target = -1
						return False
					
					self.PN_INS_MidCourse = 0
					self.LOAL_to_PN_Search_Time = 0
					
				elif not self._vls_init:
					# VLS launch attitude initialization - TVC/jet vane control for ESSM
					PN_Track = self.VLS_Init_Turn
					
					if not self.pitch_done:
						self.pitch_load = max( -0.5, self.pitch_load - 0.04 )
						p_y = math.AngleDiff( self.CurrentAimingPitch, 0.0 )
						
						self.VLS_Init_Turn.Y -= 30.0
						
						if p_y > 1.2:
							self.Constant_Drag_Factor = 2.4
						else:
							self.Constant_Drag_Factor = 0.025
						
						if math.AngleDiff( self.CurrentAimingPitch, 0.0 ) <= 0.6:
							self.pitch_done = 1
					else:
						self.pitch_load = min( 0.0, self.pitch_load + 0.03 )
						
						if self.pitch_load < 0.0:
							self.PN_Update_Ticket = 1
							return True
						
						# wic.game.Players[1].ChatMessage("fndebug: maneuver complete  p_l = %s  cDf = %s  t = %s" % ( self.pitch_load, self.Constant_Drag_Factor, msl_run_time ) )
						self._vls_init = True
						
				else:
					# En-route mode
					try:
						msl_to_track = MC_MDC_INS[self.Host.Id]
						rpn_id = ecl[msl_to_track].ReturnPPNS()
					except:
						self.PN_Exemption_EO_Limit = 1
						return False
					
					# Confirm receiving mid-course update from _ecpd
					if ecl[msl_to_track].Dequeue_INS_Update_Ticket() and self.PN_TGO > 0 and self.MDC_TWS_Updated_Position is not -1:
						self.PN_Terminal_Basket = math.Vector3( self.MDC_TWS_Updated_Position )
					
					# Loft logic:  There is no lofting for ESSM; we will revisit for SM-6.
					PN_Track = self.PN_Terminal_Basket
			else:
				## 
				# Terminal homing state
				if not aegis_maintain_terminal_lock():
					if self.PN_INS_Cache is not -1:
						PN_Track = self.PN_INS_Cache
					else:
						return False
				else:
					try:
						PN_Track = wic.game.Units[ self.tac_target ].Position
						self.PN_INS_Cache = math.Vector3(PN_Track)
					except:
						# wic.game.UnknownUnitException
						return False
			
			# Homing Loop
			if self.PN_Pos_Old and self.PN_Tgt_Pos_Old:
				myPos_Old = self.PN_Pos_Old
				myPos_New = math.Vector3( self.current_state )
				targetPos_Old = self.PN_Tgt_Pos_Old
				targetPos_New = math.Vector3( PN_Track )
				
				self.PN_Target_Direction_Vector = math.Vector3( targetPos_New ) - targetPos_Old
				LOS_old = ( targetPos_Old - myPos_Old )
				LOS_new = ( targetPos_New - myPos_New )
				
				if Ballistic_Loft:
					# Since we are using static boost height, we do not want to use LOS_new to generate Rtm
					Rtm = ( math.Vector3( PN_Track ) - myPos_New ).Length()
					Vc = ( Rtm - LOS_old.Length() )/EXFLINT_TICKTOCK
				else:
					Rtm = LOS_new.Length()
					Vc = ( Rtm - LOS_old.Length() )/EXFLINT_TICKTOCK
				
				try:
					v_tgo = min( Rtm / abs(Vc), 17.0 )
				except ZeroDivisionError:
					# This should never happen!
					if self.PN_INS_MidCourse or self.m_type__msl != 6:
						wcc_panic( "_awsd", "ex_guidance", 10927)
					
					return False
				
				dv1 = ( targetPos_New - targetPos_Old ).Length()
				# 0.32406/EXFLINT_TICKTOCK = acceleration due to gravity (EXFLINT_G/dT)
				nT = ( dv1 - self.dv0 ) + 0.32406
				self.dv0 = dv1
				
				self.PN_TGO = v_tgo
				
				LOS_old.NormalizeSafe()
				LOS_new.NormalizeSafe()
				
				if self.PN_INS_MidCourse:
					# Enroute navigation:  Commanded acceleration is LOS
					self.PN_Required_Direction_Add = LOS_new
				else:
					# Proportional navigation:  Commanded acceleration is normal to LOS
					if LOS_old is math.Vector3(0, 0, 0):
						LOS_Delta = math.Vector3( 0, 0, 0 )
						LOS_Rate = 0.0
					else:
						LOS_Delta = math.Vector3( LOS_new ) - LOS_old
						LOS_Rate = LOS_Delta.Length()
					
					# Closing velocity (Vc) is not supposed to be enlarging. If so, we're unable to
					# meet acceleration command to intercept (insufficient energy)
					if Vc > 0 and self.miss_enlarging is not -1:
						self.miss_enlarging += 1
					
					# Navigation constant
					N = 3.0
					
					# Instantaneous closing velocity (Vc) is -LOS_Rate
					normal = LOS_Delta * max( nT/EXFLINT_TICKTOCK, 27.02 ) * (0.5 * N)
					self.PN_Required_Direction_Add = LOS_new * N * -LOS_Rate * LOS_Rate + normal
					
				# Let EXFLINT_Integrate() know that we're commanding latax
				self.PN_Update_Ticket = 1
			
			# end homing loop
			self.PN_Pos_Old = math.Vector3( self.current_state )
			self.PN_Tgt_Pos_Old = math.Vector3( PN_Track )
			
			return True
			#### end ex_guidance() ####
		
		def ex_warhead_fuze():
			global MC_SARH
			
			if self.PN_INS_MidCourse:
				return False
			
			try:
				mc_key = MC_SARH[wic.game.Units[self.tac_target].Id]
			except:
				pass
			
			def report_kill():
				global ecl, _isr_aws_tao
				
				try:
					ecl[mc_key].set_detonated()
					
					_isr_aws_tao = ( wic.common.GetCurrentTime(), 4, mc_key )
				except Exception, e:
					pass
			
			try:
				c_vect = ( math.Vector3( wic.game.Units[self.tac_target].Position ) - self.Host.Position )
				rtm = c_vect.Length()
				
				if rtm <= 15.0:
					c_vect.NormalizeSafe()
					fb = c_vect.Dot( math.Vector3( self.Total_Velocity_Vector ).NormalizeSafe() )
					
					# Proximity scanning beam mutations
					if rtm <= 7.0 and not self.warhead_air_mode and fb <= 1.0:
						wic.game.Units[self.tac_target].Invulnerable = False
						report_kill()
						return True
					
					if fb < -0.5:
						wic.game.Units[self.tac_target].Invulnerable = False
						report_kill()
						return True
			except:
				pass
			
			return False
			
			
		# Initialize wicg
		ex_socket()
		
		accumulator = 0.0
		if wic.common.GetElapsedTime() > 0.037:
			accumulator += wic.common.GetElapsedTime()
		else:
			accumulator += EXFLINT_TICKTOCK
		
		# Lerp loop
		while accumulator >= EXFLINT_TICKTOCK:
			self.prev_state = math.Vector3( self.current_state )
			
			# Move the rocket forward
			ex_propulsion()
			
			# Run the inertial measurement unit
			WICO_FLINT__IMU()
			
			# Make sure we don't crash the game
			if not ex_assert():
				WICO_FLINT__Terminate()
				return False
			
			# Maintain data-link socket to ground radar station (RS)
			MC_IBCS_ECP_MDC_Update()
			
			# Guidance
			ex_guidance()
			
			# proxmity fuse
			if ex_warhead_fuze():
				return True
			
			# FLINT50 1103
			if self.Prop_Steering_Tic is 1:
				# Snap physics to missile velocity vector
				
				# EXFLINT65
				if self.PN_Update_Ticket is 1:
					Heading_Frame = math.Vector3( self.Total_Velocity_Vector ) + math.Vector3( self.PN_Required_Direction_Add ).NormalizeSafe()
				else:
					Heading_Frame = math.Vector3( self.Total_Velocity_Vector ) * 2.0
				
				# omviz_head, omviz_pitch
				self.qt_axis_data[0] = wic.common.math.VectorToAngle( Heading_Frame.X, Heading_Frame.Z )
				self.qt_axis_data[1] = -math.asin( Heading_Frame.Y / Heading_Frame.Length() ) - self.pitch_load
				
				EXFLINT_Orientation( self.PN_Update_Ticket, self.qt_axis_data, 0.087, 0.0064 )
				
				new_heading = self.qt_axis_data[6]
				new_pitch = self.qt_axis_data[7]
				
				self.Host.SetBodyHeading( new_heading )
				self.CurrentAimingHeading = math.CropAngle( new_heading - self.Host.BodyHeading )
				
				self.AimingForPitch = new_pitch
				self.CurrentAimingPitch = new_pitch
								
				self.Prop_Steering_Tic = 0
			
			accumulator -= EXFLINT_TICKTOCK
			self.i_time += EXFLINT_TICKTOCK
			
		# interpolate
		alpha = float( accumulator / EXFLINT_TICKTOCK )
		self.Host.Position = ( self.current_state * alpha) + ( self.prev_state * (1.0 - alpha) )
		
		return False

	def FireProjectile( self ):
		return True
