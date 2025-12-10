import pyads

PLC_AMS = "5.89.239.104.1.1" 
PLC_IP = "169.254.215.116"

with pyads.Connection(PLC_AMS, 851, PLC_IP) as plc:
    plc.open()  # <- Klammern!

    print("State:", plc.read_state())
    plc.write_by_name("GVL.mwFrStart", True,pyads.PLCTYPE_BOOL)
    print(plc.read_by_name("GVL.mwFrStart",pyads.PLCTYPE_BOOL))
    plc.get("GVL.mwFrStart")

plc.close() # <- Klammern!
