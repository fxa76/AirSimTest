from msgobj.ThreadSafeSingleton import SingletonMeta


class CancellationToken(metaclass=SingletonMeta):

    def __init__(self) -> None:
        self.is_cancelled = False

    def cancel(self):
        self.is_cancelled = True
