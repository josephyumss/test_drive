class FaultInjector:
    """Deterministic publication and value-fault helper."""

    def __init__(self) -> None:
        self._disabled_sources: set[str] = set()
        self._frozen_values: dict[str, float] = {}

    def disable(self, source: str) -> None:
        self._disabled_sources.add(source)

    def enable(self, source: str) -> None:
        self._disabled_sources.discard(source)

    def should_publish(self, source: str) -> bool:
        return source not in self._disabled_sources

    def freeze(self, field: str, value: float) -> None:
        self._frozen_values[field] = value

    def clear_freeze(self, field: str) -> None:
        self._frozen_values.pop(field, None)

    def value(self, field: str, normal_value: float) -> float:
        return self._frozen_values.get(field, normal_value)
