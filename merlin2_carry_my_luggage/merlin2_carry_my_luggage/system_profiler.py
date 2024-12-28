# Copyright (C) 2024 Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import csv
import time
import psutil
import threading


class SystemProfiler(threading.Thread):

    def __init__(self, interval: int = 1, filename: str = "system_profile.csv") -> None:

        super().__init__()

        self.interval = interval
        self.filename = filename
        self.running = True
        self.fieldnames = [
            "timestamp",
            "cpu_usage",
            "ram_usage",
            "bytes_sent",
            "bytes_recv",
            "packets_sent",
            "packets_recv",
        ]

        with open(self.filename, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.fieldnames)
            writer.writeheader()

    def run(self) -> None:

        prev_net_io = psutil.net_io_counters()

        while self.running:

            cpu_usage = psutil.cpu_percent(interval=self.interval)
            ram_usage = psutil.virtual_memory().percent
            net_io = psutil.net_io_counters()

            data = {
                "timestamp": time.time(),
                "cpu_usage": cpu_usage,
                "ram_usage": ram_usage,
                "bytes_sent": net_io.bytes_sent - prev_net_io.bytes_sent,
                "bytes_recv": net_io.bytes_recv - prev_net_io.bytes_recv,
                "packets_sent": net_io.packets_sent - prev_net_io.packets_sent,
                "packets_recv": net_io.packets_recv - prev_net_io.packets_recv,
            }
            prev_net_io = net_io

            with open(self.filename, "a", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=self.fieldnames)
                writer.writerow(data)

    def stop(self) -> None:
        self.running = False
