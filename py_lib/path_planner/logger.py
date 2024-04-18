class Logger:
    def __init__(self, filename="path_history.txt"):
        self.filename = filename

    def log(self, msg, do_print=True):
        if do_print:
            print(msg)
        with open(self.filename, "a") as file:
            file.write(f"{msg}\n")
           
