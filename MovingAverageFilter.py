class MovingAverageFilter:
    """A simple moving average filter for smoothing data."""

    __slots__ = ("window_size", "_buffer", "_count", "_index", "_sum")

    def __init__(self, window_size):
        """Initialize the moving average filter with a specified window size."""
        window_size = int(window_size)
        if window_size <= 0:
            raise ValueError("window_size must be positive")

        self.window_size = window_size
        self._buffer = [0.0] * window_size
        self._count = 0
        self._index = 0
        self._sum = 0.0

    def update(self, new_value):
        """Update the filter with a new value and return the current moving average."""
        value = float(new_value)
        buffer = self._buffer
        index = self._index
        total = self._sum
        window_size = self.window_size
        count = self._count

        if count < window_size:
            buffer[index] = value
            total += value
            count += 1
            index += 1
            if index == window_size:
                index = 0

            self._index = index
            self._count = count
            self._sum = total
            return total / count

        total += value - buffer[index]
        buffer[index] = value
        index += 1
        if index == window_size:
            index = 0

        self._index = index
        self._sum = total
        return total / window_size

    def moving_average(self, new_value):
        """Convenience method to update the filter and get the current moving average."""
        return self.update(new_value)

    @property
    def value(self):
        count = self._count
        if count == 0:
            return 0.0
        return self._sum / count


class LowPassFilter:
    """A simple low-pass filter for smoothing data."""
    __slots__ = ("cutoff_hz", "_value", "_initialized")

    def __init__(self, cutoff_hz):
        cutoff_hz = float(cutoff_hz)
        if cutoff_hz <= 0.0:
            raise ValueError("cutoff_hz must be positive")

        self.cutoff_hz = cutoff_hz
        self._value = 0.0
        self._initialized = False

    def update(self, new_value, dt):
        value = float(new_value)
        dt = max(0.0, float(dt))

        if not self._initialized or dt <= 0.0:
            self._value = value
            self._initialized = True
            return self._value

        rc = 1.0 / (2.0 * 3.141592653589793 * self.cutoff_hz)
        alpha = dt / (rc + dt)
        self._value = self._value + alpha * (value - self._value)
        return self._value

    @property
    def value(self):
        return self._value
