class StatusText:
    def __init__(self):
        self.text = None

    def update(self, mavpackeftype_statustext: dict):
        # print("Statustext updated : {}".format(mavpackeftype_statustext))
        self.text = mavpackeftype_statustext['text']

