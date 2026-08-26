import logging
from pathlib import Path
from datetime import datetime


class LoggerManager:
    _initialized = False

    @classmethod
    def initialize(cls, app_name="project_a7", print_on_console=True):
        if cls._initialized:
            return

        Path("logs").mkdir(exist_ok=True)

        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        log_file = Path("logs") / f"{app_name}_{timestamp}.log"

        formatter = logging.Formatter(
            "%(asctime)s | %(levelname)s | %(name)s | %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S"
        )

        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(formatter)

        root_logger = logging.getLogger()
        root_logger.setLevel(logging.INFO)
        root_logger.addHandler(file_handler)

        if print_on_console:
            console_handler = logging.StreamHandler()
            console_handler.setFormatter(formatter)
            root_logger.addHandler(console_handler)

        cls._initialized = True