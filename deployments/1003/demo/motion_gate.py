import threading,time,secrets,math
class MotionGate:
    def __init__(self,clock=time.monotonic):
        self.clock=clock;self.mutex=threading.Lock();self.execution=threading.RLock();self.nonce=secrets.token_hex(16)
    def ticket(self):return {'nonce':self.nonce,'deadline':self.clock()+10.}
    def invalidate(self):
        with self.execution:self.nonce=secrets.token_hex(16)
    def validate(self,ticket):
        try:deadline=float(ticket['deadline']);nonce=ticket['nonce']
        except (KeyError,TypeError,ValueError):raise ValueError('请刷新网页后重新发送指令')
        if nonce!=self.nonce or not math.isfinite(deadline) or not 0<deadline-self.clock()<=10.1:
            raise ValueError('启动请求已过期或已被停止指令作废，请重新操作')
    def reserve(self,ticket):
        self.validate(ticket)
        if not self.mutex.acquire(blocking=False):raise ValueError('另一条启动请求正在准备，不会重复排队')
        try:self.validate(ticket)
        except Exception:self.mutex.release();raise
