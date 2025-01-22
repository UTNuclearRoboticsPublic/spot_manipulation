import time
import numpy as np

class TrajectoryManager:
    def __init__(self, points, times_since_ref, ref_time: float):
        self.points = list(points)
        self.times_since_ref = list(times_since_ref)
        self.ref_time: float = ref_time
        self._last = False
        
        if (len(self.times_since_ref) != len(self.points)):
            raise Exception("Trajecory Manager must take an equal number of points and timestamps")

        self.size = len(self.points)
            

    def get_window(self, window_size) -> tuple[list, list[float], float]:
        """
        Get a set number of points from the trajectory based on the current time since reference
        
        Args:
            window_size: How many points to retrieve. The actual number returned may be less
            if there aren't enough points remaining until the end
        Returns:
            List: trajectory points in the current window
            List: timestamps corresponding to the trajectory points
            float: Time until this window expires in seconds. Zero if the trajectory has finished
        """

        # Calculate where we are on the trajectory
        current_time_since_ref = max(time.time() - self.ref_time, 0.0)

        # Get the index of the point after the one that was most recently executed
        active_index = self.size
        for idx, time_since_ref in enumerate(self.times_since_ref):
            if (time_since_ref - current_time_since_ref > 0.15): # Add a cushion of a small fraction of a second to account for transmission times
                active_index = idx
                break

        # Get the final index in this window
        end_index = min(active_index + window_size, self.size)

        # Get the time until this window expires
        if end_index != active_index:
            expiration_time = self.times_since_ref[end_index-1] - current_time_since_ref  
        else:
            expiration_time = 0

        # Record whether or not this is the final segment of the trajectory
        if end_index == self.size:
            self._last = True

        return self.points[active_index:end_index], self.times_since_ref[active_index:end_index], expiration_time

    def done(self) -> bool:
        """
        Return whether or the full trajectory time has elapsed
        """
        if self.size == 0: 
            return True

        return time.time() > self.ref_time + self.times_since_ref[-1] 
    
    def last(self) -> bool:
        """
        Return whether or not the most recent window is the last on in the trajectory
        """
        return self._last