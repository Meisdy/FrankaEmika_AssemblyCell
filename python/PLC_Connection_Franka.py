import os
import json
import pyads


import time
import pyads

SIM_FILE = "plc_data.json"

class PLC_Connection_Franka:
    def __init__(self, plc_ip, ams_net_id, ams_port, simulate=False, sim_file=SIM_FILE):
        self.simulate = simulate
        self.sim_file = sim_file

        if not simulate:
            try:
                self.plc = pyads.Connection(ams_net_id, ams_port, plc_ip)
                self.plc.open()
                print("✅ Connected to PLC")
            except Exception as e:
                print(f"⚠️ Could not connect to PLC ({e}). Switching to simulation mode.")
                self.simulate = True
        else:
            print("⚙️ Running in simulation mode (JSON file).")

        # Create file if missing
        if self.simulate and not os.path.exists(sim_file):
            with open(sim_file, "w") as f:
                json.dump({}, f, indent=4)

    # -------------------------------------------------
    def _get_symbol(self, name):
        if not hasattr(self, "_symbol_cache"):
            self._symbol_cache = {}
        if name not in self._symbol_cache:
            self._symbol_cache[name] = self.plc.get_symbol(f"GVL.{name}")
        return self._symbol_cache[name]

    # -------------------------------------------------
    def get(self, *var_names: str):
        if self.simulate:
            data = self._read_sim_file()
            return [data.get(name, None) for name in var_names]
        else:
            return [self._get_symbol(name).read() for name in var_names]

    def set(self, **var_values):    #set(var_name=True/12.4/"What ever")
        print("Set function")
        if self.simulate:
            data = self._read_sim_file()
            data.update(var_values)
            self._write_sim_file(data)
        else:
            for name, value in var_values.items():
                self._get_symbol(name).write(value)

    # -------------------------------------------------
    # Simulation File I/O (JSON)
    # -------------------------------------------------
    def _read_sim_file(self):
        if not os.path.exists(self.sim_file):
            return {}
        with open(self.sim_file, "r") as f:
            try:
                return json.load(f)
            except json.JSONDecodeError:
                return {}

    def _write_sim_file(self, data):
        with open(self.sim_file, "w") as f:
            json.dump(data, f, indent=4)




# ==========================================================
# MAIN
# ==========================================================
if __name__ == "__main__":
    plc = PLC_Connection_Franka(
        ams_net_id="192.168.10.1.1.1",
        plc_ip="192.168.10.1",
        ams_port=851
    )

    while True:
        mwFrStart, mwFrRefill, mwFrFinished, mwFrStatus, mwFrVisiCode, mwAssemblyColor = plc.get("mwFrStart", "mwFrRefill", "mwFrFinished", "mwFrStatus", "mwFrVisiCode", "mwAssemblyColor")
        if not mwFrStart:
            continue
        else:
            
            plc.set(mwFrStart=False)
            # go through the program
            # set Franka status
            plc.set(mwFrStatus="Get Container")
            plc.set(mwFrStatus="Assemble ...")
            plc.set(mwFrStatus="Place on AGV")


            plc.set(mwAssemblyColor="blue")
            plc.set(mwFrFinished=True)
            plc.set(mwFrRefill=True)
            plc.set(mwFrVisiCode=[111, 242, 313, 214, 145, 126])

            

            # After work is done
            
            continue
               

# class PLCConnection:
#     # ---------------- PLC SETTINGS ----------------
#     PLC_IP = "169.254.215.116"
#     AMS_NET_ID = "5.89.239.104.1.1"
#     AMS_PORT = 851

#     VAR_START = "GVL.IxFrStart"
#     VAR_DONE = "GVL.QxVFrFinished"

#     # ============================================================
#     @staticmethod
#     def connectPLC():
#         """Create and return an open PLC connection"""
#         plc = pyads.Connection(
#             PLCConnection.AMS_NET_ID,
#             PLCConnection.AMS_PORT,
#             PLCConnection.PLC_IP,
#         )
#         plc.open()
#         print("✅ Connected to PLC")
#         return plc

#     # ============================================================
#     @staticmethod
#     def wait_for_start_signal():
#         """Wait for PLC start signal (IxV<Start = TRUE)."""
#         plc = PLCConnection.connectPLC()
#         print("⏳ Waiting for start signal from PLC...")

#         try:
#             while True:
#                 start = plc.read_by_name(PLCConnection.VAR_START, pyads.PLCTYPE_BOOL)
#                 if start:
#                     print("🚀 Start signal received from PLC!")
#                     plc.write_by_name(PLCConnection.VAR_START, False, pyads.PLCTYPE_BOOL)
#                     return True
#                 time.sleep(0.05)
#         finally:
#             plc.close()
#             print("🔌 PLC connection closed after waiting.")

#     # ============================================================
#     @staticmethod
#     def send_done_signal():
#         """
#         Send detected object information (as strings)
#         and set done signal (QxViDone = TRUE for one cycle).
#         """
#         plc = PLCConnection.connectPLC()

#         try:
#             # Send array of string info
#             var_name = f"GVL.IxFrStart[{i}]"
#             plc.write_by_name(PLCConnection.VAR_DONE, True, pyads.PLCTYPE_BOOL)
#             print(f"📤 Sent {"IxFrStarted"}

#             # Send Done signal
#             plc.write_by_name(PLCConnection.VAR_DONE, True, pyads.PLCTYPE_BOOL)
#             print("📤 Set QxFRFinished = TRUE")
#             time.sleep(0.2)
#             plc.write_by_name(PLCConnection.VAR_DONE, False, pyads.PLCTYPE_BOOL)
#             print("🔁 Reset QxFRFinished  = FALSE")

#         finally:
#             plc.close()
#             print("🔌 PLC connection closed after sending results.")

