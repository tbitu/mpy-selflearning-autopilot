class GPSStub:
    def __init__(self):
        pass

    def update(self):
        """Read data from UART (Stub)"""
        pass

    @property
    def has_fix(self):
        return False

    @property
    def cog(self):
        """Course Over Ground"""
        return None

    @property
    def sog(self):
        """Speed Over Ground"""
        return None
        
    @property
    def lat(self):
        return None
        
    @property
    def lon(self):
        return None
