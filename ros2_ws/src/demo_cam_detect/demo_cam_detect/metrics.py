class EmaMeters:
    def __init__(self, alpha=0.1):
        self.a = alpha
        self.v = {"pre":0.0, "infer":0.0, "post":0.0, "total":0.0}
        self.init = False

    def update(self, pre, infer, post, total):
        if not self.init:
            self.v = {"pre":pre, "infer":infer, "post":post, "total":total}
            self.init = True
            return
        for k,val in [("pre",pre),("infer",infer),("post",post),("total",total)]:
            self.v[k] = (1-self.a)*self.v[k] + self.a*val

    def ema(self):
        return dict(self.v)
