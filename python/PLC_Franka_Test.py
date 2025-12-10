import time
import pyads

PLC_IP = "169.254.215.116"
AMS_NET_ID = "5.89.239.104.1.1"
AMS_PORT = 851
PLC_AMS = "192.168.10.1.1.1"
PLC_IP = "192.168.10.1"
# --------------------------------------
# PLC VARIABLE DEFINITIONS (Your list)
# --------------------------------------
VAR_IxFrStart      = "GVL.IxFrStart"       # BOOL (PLC → Python)
VAR_QxFrFinished   = "GVL.QxFrFinished"    # BOOL (Python → PLC)
VAR_mwFrStatus     = "GVL.mwFrStatus"      # INT  (PLC → Python)
VAR_mxFrTaskCode   = "GVL.mxFrTaskCode"    # ARRAY [1..4] OF INT
VAR_QxFrEmergency  = "GVL.QxFrEmergency"   # BOOL (PLC → Python)
VAR_IxFrRefilled   = "GVL.IxFrRefilled"    # BOOL (PLC → Python)
VAR_QxFrRefill     = "GVL.QxFrRefill"      # BOOL (Python → PLC)


def connect():
    plc = pyads.Connection(AMS_NET_ID, AMS_PORT, PLC_IP)
    plc.open()
    print("\n✅ Connected to PLC\n")
    return plc


# ---------------------------------------------------------
# TEST FUNCTION — READ + WRITE ALL VARIABLES
# ---------------------------------------------------------
def test_all_variables():
    plc = connect()

    try:
        print("=============================================")
        print("🔍 READING ALL PLC → PYTHON SIGNALS")
        print("=============================================")

        ix_start    = plc.read_by_name(VAR_IxFrStart, pyads.PLCTYPE_BOOL)
        print(ix_start)
        return
        fr_status   = plc.read_by_name(VAR_mwFrStatus, pyads.PLCTYPE_INT)
        emergency   = plc.read_by_name(VAR_QxFrEmergency, pyads.PLCTYPE_BOOL)
        refilled    = plc.read_by_name(VAR_IxFrRefilled, pyads.PLCTYPE_BOOL)

        task_array = []
        for i in range(1, 5):
            val = plc.read_by_name(f"{VAR_mxFrTaskCode}[{i}]", pyads.PLCTYPE_INT)
            task_array.append(val)

        print(f"IxFrStart     : {ix_start}")
        print(f"mwFrStatus    : {fr_status}")
        print(f"QxFrEmergency : {emergency}")
        print(f"IxFrRefilled  : {refilled}")
        print(f"mxFrTaskCode  : {task_array}")

        print("\n=============================================")
        print("✍️ WRITING ALL PYTHON → PLC SIGNALS")
        print("=============================================")

        # ✅ Test QxFrFinished toggle
        plc.write_by_name(VAR_QxFrFinished, True, pyads.PLCTYPE_BOOL)
        print("Wrote TRUE → QxFrFinished")
        time.sleep(0.2)
        plc.write_by_name(VAR_QxFrFinished, False, pyads.PLCTYPE_BOOL)
        print("Wrote FALSE → QxFrFinished")

        # ✅ Test QxFrRefill toggle
        plc.write_by_name(VAR_QxFrRefill, True, pyads.PLCTYPE_BOOL)
        print("Wrote TRUE → QxFrRefill")
        time.sleep(0.2)
        plc.write_by_name(VAR_QxFrRefill, False, pyads.PLCTYPE_BOOL)
        print("Wrote FALSE → QxFrRefill")

        # ✅ Test writing to the task array
        test_values = [11, 22, 33, 44]
        for i, val in enumerate(test_values, start=1):
            plc.write_by_name(f"{VAR_mxFrTaskCode}[{i}]", val, pyads.PLCTYPE_INT)
            print(f"Wrote {val} → mxFrTaskCode[{i}]")

        print("\n✅ ALL VARIABLES TESTED SUCCESSFULLY")

    except Exception as e:
        print("❌ ERROR:", e)

    finally:
        plc.close()
        print("\n🔌 PLC Connection Closed\n")


# ---------------------------------------------------------
# RUN TEST
# ---------------------------------------------------------
if __name__ == "__main__":
    test_all_variables()
