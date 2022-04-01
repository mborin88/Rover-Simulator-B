import math

class PController:
    """
    A proportional controller class.
    """
    def __init__(self, ref=None, gain=None):
        self._ref = ref     # The reference point.
        self._gain = gain   # The gain, Kp.

    def set_ref(self, new_ref):
        """
        Change reference point.
        """
        self._ref = new_ref

    def set_gain(self, new_gain):
        """
        Change gain value.
        """
        self._gain = new_gain

    def execute(self, controlled_object):
        """
        Apply proportional control effect given controlled object.
        """
        r = self._ref                   # Reference
        x = controlled_object           # State variable(s)
        e = []                          # Error
        for i in range(len(x)):
            e.append(r[i] - x[i])       # Control input
        
        kp = self._gain                # Gain
        u = 0.0  
        for j in range(len(e)):
            u += kp[j] * e[j]
        return u

    def execute2(self, controlled_object):
        """
        Apply proportional control effect given controlled object.
        """
        r = self._ref                   # Reference
        x = controlled_object           # State variable(s)
        dist = math.sqrt((x[0] - r[0])**2 + (x[1] - r[1])**2)   # Absolute distance
        kp = self._gain                # Gain
        u = 0.0  
        u = dist * kp[1]

        return u
