class SensorTimeoutMonitor:
    def __init__(self, timeouts_s: dict[str, float]) -> None:
        if not timeouts_s or any(value <= 0 for value in timeouts_s.values()):
            raise ValueError("all sensor timeouts must be positive")
        self.timeouts_s = dict(timeouts_s)
        self.last_update_s: dict[str, float] = {}

    def update(self, source: str, now_s: float) -> None:
        if source not in self.timeouts_s:
            raise KeyError(source)
        self.last_update_s[source] = now_s

    def has_seen(self, source: str) -> bool:
        return source in self.last_update_s

    def is_timed_out(self, source: str, now_s: float) -> bool:
        if source not in self.timeouts_s:
            raise KeyError(source)
        if source not in self.last_update_s:
            return False
        elapsed = now_s - self.last_update_s[source]
        return elapsed > self.timeouts_s[source] + 1e-9

    def all_seen(self, *sources: str) -> bool:
        return all(self.has_seen(source) for source in sources)
