import yaml
import os
from typing import Dict, Any

class ConfigLoader:
    def __init__(self, config_path: str):
        self.config_path = config_path

    def load_scenario(self, scenario_name: str) -> Dict[str, Any]:
        """
        Loads the YAML file and returns the specific scenario dictionary.
        Raises an explanation if the file or scenario is not found.
        """
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"Configuration file not found: {self.config_path}")
            
        with open(self.config_path, "r") as file:
            try:
                data = yaml.safe_load(file)
            except yaml.YAMLError as exc:
                raise ValueError(f"Error parsing YAML file: {exc}")

        scenarios = data.get("scenarios", {})
        if scenario_name not in scenarios:
            raise KeyError(f"Scenario '{scenario_name}' not found in {self.config_path}")

        return scenarios[scenario_name]
