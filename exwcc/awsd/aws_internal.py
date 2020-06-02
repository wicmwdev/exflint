# module import
import wic.common.math as math
import wic

class aws_internal(object):
	def __init__(self, aHost):
		# constructor
		self.spg62 = {1:[], 2:[], 3:[]}
		self.directions = {1:None, 2:None, 3:None}
		self.host = aHost
		self._tdm_time = {1:None, 2:None, 3:None}
		self._i_tracks = {}
		
	def allocate(self, aKey, aTimeSlot = 2.5):
		# allocate a channel for schedule
		
		if aKey in self._i_tracks:
			return True
		
		try:
			_myself = math.Vector3( wic.game.Units[self.host].Position )
			_target = math.Vector3( wic.game.Units[aKey].Position )
			radar_to_target = math.Vector3(_myself) - _target
		except:
			return False
		
		for i in self.spg62:
			boresight_pass = False
			
			# walk TDM scheduler
			if self.spg62[i] and self._tdm_time[i] and ( wic.common.GetCurrentTime() - self._tdm_time[i] ) <= aTimeSlot:
				continue
			
			# mcu limits
			if len(self._i_tracks) >= 48:
				continue
			
			if self.directions[i]:
				try:
					radar_facing = math.Vector3(_myself) - self.directions[i]
					
					rd_to_tgt_heading = math.atan2( radar_to_target.X, radar_to_target.Z )
					rd_boresight_heading = math.atan2( radar_facing.X, radar_facing.Z )
					
					if math.AngleDiff( rd_to_tgt_heading, rd_boresight_heading ) <= 1.0472:
						boresight_pass = True
					
					# wic.game.Players[1].ChatMessage("aw_debug: boresight = %s > 1.04" % math.AngleDiff( rd_to_tgt_heading, rd_boresight_heading ) )
				except:
					continue
			else:
				boresight_pass = True
			
			if not boresight_pass:
				continue
			
			# schedule illuminator resource
			self.spg62[i].append(aKey)
			self.directions[i] = math.Vector3(_target)
			self._tdm_time[i] = wic.common.GetCurrentTime()
			self._i_tracks[aKey] = None
			return True
		
		return False
	
	def free(self, aKey):
		if aKey in self._i_tracks:
			del self._i_tracks[aKey]
		
		for i in self.spg62:
			if aKey in self.spg62[i]:
				self.spg62[i].remove(aKey)
				
				if len(self.spg62[i]) < 1:
					self.directions[i] = None
				
				return True
		
		return False
	
	def get_direction(self, spg62):
		# called by wicg driver for awsd to drive aim heading & pitch for SPG-62 illuminators
		if spg62 in self.directions:
			return self.directions[spg62]
		else:
			raise TypeError
	
	def illuminate(self, aKey):
		# called by interceptor to request target illumination
		if aKey in self._i_tracks:
			self._i_tracks[aKey] = wic.common.GetCurrentTime()
			return True
		
		return False
	
	def waveform(self, aKey):
		# called by interceptor to check SARH illumination
		if ( aKey in self._i_tracks and self._i_tracks[aKey] and
			( wic.common.GetCurrentTime() - self._i_tracks[aKey] ) < 0.2 ):
				return True
		
		return False
	
	def run(self, mcu_dict = {}):
		# called by _awsd to run illumination beams
		remove_list = []
		for i in self._i_tracks:
			try:
				t_pos = wic.game.Units[i].Position
			except:
				remove_list.append(i)
				continue
			
			mcu_dict[i] = self.host
		
		for i in remove_list:
			self.free(i)
		
		# illuminate one track for each illuminator
		for anten in self.spg62:
			for track in self.spg62[anten]:
				if track in self._i_tracks and self._i_tracks[track]:
					self.directions[anten] = math.Vector3(t_pos)
					self._i_tracks[track] = wic.common.GetCurrentTime()
					# antenna occupied, break out of inner loop to move to next track
					break
		
		return None
	
	def get_tracks(self):
		return self._i_tracks
	
	def get_xmits(self):
		return self.spg62
	
