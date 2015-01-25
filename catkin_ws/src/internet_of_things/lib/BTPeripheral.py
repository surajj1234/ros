class BTPeripheral():

    def __init__(self):

        self.localName = "Unknown"
        
        self.btID = "FF:FF:FF:FF:FF:FF"
        self.btAddrType = 0x00      # Public by default
        
        self.connectable = True
        self.undirected = True
        self.scannable = False

        self.advData = []
        self.scanRspData = []
