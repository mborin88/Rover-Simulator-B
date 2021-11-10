class Packet:
    """
    A packet class representing a transmission.
    """
    def __init__(self, tx, payload):
        self._tx = tx               # The transmitter, a radio object.
        self._payload = payload     # The payload, a list.

    @property
    def tx(self):
        return self._tx

    @property
    def payload(self):
        return self._payload
