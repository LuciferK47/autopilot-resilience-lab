import pandas as pd
from pymavlink import DFReader
from pathlib import Path

class EKFForensics:
    """
    Analyzes EKF data from ArduPilot DataFlash (.BIN) logs.
    Extracts, aligns, and merges multi-rate EKF messages into a synchronous feature matrix.
    """

    def __init__(self, filepath: Path) -> None:
        """
        Initializes the EKFForensics analyzer.
        
        Args:
            filepath (Path): Path to the .BIN DataFlash log file.
        """
        self.filepath = Path(filepath)
        if not self.filepath.exists():
            raise FileNotFoundError(f"DataFlash log file not found: {self.filepath}")

    def extract_ekf_data(self, fault_time_us: int) -> pd.DataFrame:
        """
        Extracts EKF and motor output telemetry from the DataFlash log.
        
        Iterates the log exactly once, collecting:
        - XKF1: Physical drift state (TimeUS, Lat, Lng, Alt)
        - XKF3: Raw innovations (TimeUS, IVN, IVE, IVD, IPN, IPE)
        - XKF4: Normalized test ratios (TimeUS, SV, SP, SH, SM)
        - RCOU: Motor output saturation (TimeUS, C1, C2, C3, C4)
        
        Returns:
            pd.DataFrame: A synchronous pandas DataFrame merged based on TimeUS using forward fill.
            
        Raises:
            ValueError: If critical messages are missing or unable to parse the log.
        """
        try:
            log = DFReader.DFReader_binary(str(self.filepath))
        except Exception as e:
            raise ValueError(f"Failed to read DataFlash log: {e}") from e

        xkf1_data = []
        xkf3_data = []
        xkf4_data = []
        rcou_data = []

        # Iterate over the log exactly once
        while True:
            # We only want to parse these specific messages to save CPU cycles
            msg = log.recv_match(type=['XKF1', 'XKF3', 'XKF4', 'RCOU'])
            if msg is None:
                break  # End of log file reached
                
            mtype = msg.get_type()
            
            if mtype == 'XKF1':
                xkf1_data.append({
                    'TimeUS': msg.TimeUS,
                    'Roll': getattr(msg, 'Roll', None),
                    'Pitch': getattr(msg, 'Pitch', None),
                    'Yaw': getattr(msg, 'Yaw', None),
                    'VN': getattr(msg, 'VN', None),
                    'VE': getattr(msg, 'VE', None),
                    'VD': getattr(msg, 'VD', None),
                    'PN': getattr(msg, 'PN', None),
                    'PE': getattr(msg, 'PE', None),
                    'PD': getattr(msg, 'PD', None),
                })
            elif mtype == 'XKF3':
                xkf3_data.append({
                    'TimeUS': msg.TimeUS,
                    'IVN': getattr(msg, 'IVN', None),
                    'IVE': getattr(msg, 'IVE', None),
                    'IVD': getattr(msg, 'IVD', None),
                    'IPN': getattr(msg, 'IPN', None),
                    'IPE': getattr(msg, 'IPE', None)
                })
            elif mtype == 'XKF4':
                xkf4_data.append({
                    'TimeUS': msg.TimeUS,
                    'SV': getattr(msg, 'SV', None),
                    'SP': getattr(msg, 'SP', None),
                    'SH': getattr(msg, 'SH', None),
                    'SM': getattr(msg, 'SM', None)
                })
            elif mtype == 'RCOU':
                rcou_data.append({
                    'TimeUS': msg.TimeUS,
                    'C1': getattr(msg, 'C1', None),
                    'C2': getattr(msg, 'C2', None),
                    'C3': getattr(msg, 'C3', None),
                    'C4': getattr(msg, 'C4', None)
                })

        if not xkf1_data and not xkf3_data and not xkf4_data:
            raise ValueError("Critical EKF3 messages (XKF1, XKF3, XKF4) are missing from the log.")

        def drop_duplicates_keep_last(df):
            if df.empty: return df
            return df.drop_duplicates(subset=['TimeUS'], keep='last')

        df_xkf1 = pd.DataFrame(xkf1_data) if xkf1_data else pd.DataFrame(columns=['TimeUS', 'Roll', 'Pitch', 'Yaw', 'VN', 'VE', 'VD', 'PN', 'PE', 'PD'])
        df_xkf3 = pd.DataFrame(xkf3_data) if xkf3_data else pd.DataFrame(columns=['TimeUS', 'IVN', 'IVE', 'IVD', 'IPN', 'IPE'])
        df_xkf4 = pd.DataFrame(xkf4_data) if xkf4_data else pd.DataFrame(columns=['TimeUS', 'SV', 'SP', 'SH', 'SM'])
        df_rcou = pd.DataFrame(rcou_data) if rcou_data else pd.DataFrame(columns=['TimeUS', 'C1', 'C2', 'C3', 'C4'])

        df_xkf1 = drop_duplicates_keep_last(df_xkf1)
        df_xkf3 = drop_duplicates_keep_last(df_xkf3)
        df_xkf4 = drop_duplicates_keep_last(df_xkf4)
        df_rcou = drop_duplicates_keep_last(df_rcou)

        try:
            # Crucial Alignment Step: Sorting by TimeUS explicitly handled by concat, sort_values and ffill
            df_merged = pd.concat([df_xkf1.set_index('TimeUS'), 
                                   df_xkf3.set_index('TimeUS'), 
                                   df_xkf4.set_index('TimeUS'), 
                                   df_rcou.set_index('TimeUS')], axis=1)
            
            df_merged = df_merged.sort_values(by='TimeUS')
            
            # Step 1 Addition: dropna() after ffill(), apply normalized Time_Since_Fault
            df_merged = df_merged.ffill().dropna()
            
            df_merged = df_merged.reset_index()
            df_merged['Time_Since_Fault'] = (df_merged['TimeUS'] - fault_time_us) / 1e6
            
            return df_merged

        except Exception as e:
            raise ValueError(f"Failed to merge dataframes: {e}") from e

