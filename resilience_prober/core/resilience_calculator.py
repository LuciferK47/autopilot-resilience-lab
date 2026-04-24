import pandas as pd
import numpy as np

class ResilienceCalculator:
    """
    Calculates resilience metrics (peak error, control saturation, resilience index)
    from an aligned EKF feature matrix logging anomalies.
    """

    def __init__(self, df: pd.DataFrame, fault_time_us: int):
        """
        Initializes the ResilienceCalculator with a cleaned/aligned DataFrame.
        """
        self.df = df
        self.fault_time_us = fault_time_us

        # --- DIAGNOSTIC BLOCK ---
        post_fault_data = self.df[self.df['Time_Since_Fault'] > 0]
        print("\n--- MATRIX DIAGNOSTICS ---")
        print(f"Total Rows Parsed: {len(self.df)}")
        print(f"Post-Fault Rows: {len(post_fault_data)}")
        if not self.df.empty:
            print(f"Log ends at Time_Since_Fault: {self.df['Time_Since_Fault'].max():.3f} seconds")
        print("--------------------------\n")

    def calculate_peak_error(self) -> float:
        """
        Calculates the maximum magnitude of the EKF position innovations 
        where Time_Since_Fault > 0.
        """
        anomaly_df = self.df[self.df['Time_Since_Fault'] > 0]
        if anomaly_df.empty:
            return 0.0

        # Compute magnitude: sqrt(IPN^2 + IPE^2)
        ipn = anomaly_df['IPN'].astype(float)
        ipe = anomaly_df['IPE'].astype(float)
        
        magnitude = np.sqrt(ipn**2 + ipe**2)
        return float(magnitude.max())

    def calculate_control_saturation(self) -> float:
        """
        Calculates the percentage of time that any motor was saturated 
        (>= 1950 or <= 1050) during the anomaly timeframe.
        """
        anomaly_df = self.df[self.df['Time_Since_Fault'] > 0]
        if anomaly_df.empty:
            return 0.0

        total_rows = len(anomaly_df)

        c1 = anomaly_df['C1'].astype(float)
        c2 = anomaly_df['C2'].astype(float)
        c3 = anomaly_df['C3'].astype(float)
        c4 = anomaly_df['C4'].astype(float)

        saturated_cond = (c1 >= 1950) | (c1 <= 1050) | \
                         (c2 >= 1950) | (c2 <= 1050) | \
                         (c3 >= 1950) | (c3 <= 1050) | \
                         (c4 >= 1950) | (c4 <= 1050)

        return float((saturated_cond.sum() / total_rows) * 100.0)

    def compute_resilience_index(self) -> float:
        """
        Computes the unified Resilience Index.
        Base 100. Deducts 5 per 1.0 peak error & 1 per 1% saturation. Clamped [0, 100].
        """
        peak_error = self.calculate_peak_error()
        saturation_pct = self.calculate_control_saturation()

        score = 100.0 - (peak_error * 5.0) - saturation_pct
        return max(0.0, min(100.0, float(score)))
