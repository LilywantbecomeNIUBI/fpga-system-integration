"""Share one freshly encoded camera preview across all HTTP viewers."""
import threading
import time


class LatestPreview:
    def __init__(self, capture, encode, clock=time.monotonic):
        self.capture, self.encode, self.clock = capture, encode, clock
        self.lock = threading.Lock()
        self.cached = None
        self.encoded_at = None

    def image(self):
        item = self.frame()
        return item[1] if item else None

    def frame(self):
        with self.lock:
            now = self.clock()
            item = self.capture.frame()
            if not item or not 0 <= now - item[2] < .5:
                self.cached = None
                return None
            # Encode each source frame once, independent of the number of viewers.
            if self.cached and now - self.cached[2] < .5 and self.cached[0] == item[0] and self.cached[2] == item[2]:
                return ((self.cached[0], self.cached[2]), self.cached[3])
            encoded = item[3] if item[3] is not None else self.encode(item[1])
            self.cached = (item[0], None, item[2], encoded)
            self.encoded_at = now
            return ((item[0], item[2]), encoded)
