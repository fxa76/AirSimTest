class TextLogger:
    def __init__(self, message_stack):
        self.message_stack = message_stack

    def log(self, text):

        self.message_stack.append(text)
        print("{} : {}".format(len( self.message_stack), text))