"""
WelfordsOnlineAlgorithm.py - Online mean and variance accumulator.

This module implements Welford's algorithm for numerically stable streaming
statistics, useful for adaptive noise estimation and runtime tuning.
"""

class WelfordsOnline:
    __slots__ = ("n", "mean", "m2")

    def __init__(self):
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0

    def update(self, x):
        n = self.n + 1
        delta = x - self.mean
        mean = self.mean + (delta / n)

        self.n = n
        self.mean = mean
        self.m2 += delta * (x - mean)
        return mean

    @property
    def variance_population(self):
        if self.n == 0:
            return 0.0
        return self.m2 / self.n

    @property
    def variance_sample(self):
        if self.n < 2:
            return 0.0
        return self.m2 / (self.n - 1)
