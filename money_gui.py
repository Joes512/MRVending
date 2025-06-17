import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Int32
import tkinter as tk
import tkinter.font as tkFont
from threading import Thread
from datetime import datetime

# ────────────────────────────  ROS node ────────────────────────────
class MoneyCountNode(Node):
    def __init__(self, gui_update_cb=None, gui_history_cb=None):
        super().__init__('money_count_node')
        self.create_subscription(Int32, '/money_count', self.listener_callback, 10)
        self.reset_publisher = self.create_publisher(Empty, '/money_reset', 10)
        self.gui_update_cb = gui_update_cb
        self.gui_history_cb = gui_history_cb
        self._last_total = 0

    def listener_callback(self, msg: Int32):
        new_total = msg.data
        if new_total < self._last_total:       # counter reset outside
            self._last_total = new_total
            if self.gui_history_cb:
                ts = datetime.now().strftime('%Y/%m/%d %H:%M:%S')
                self.gui_history_cb(f'{ts}  Reset')
            if self.gui_update_cb:
                self.gui_update_cb(new_total)
            return
        if new_total > self._last_total:       # insert coin
            delta = new_total - self._last_total
            self._last_total = new_total
            if self.gui_history_cb:
                ts = datetime.now().strftime('%Y/%m/%d %H:%M:%S')
                self.gui_history_cb(f'{ts}  Insert coin : ${delta:,}')
        if self.gui_update_cb:
            self.gui_update_cb(new_total)

    def publish_reset(self):
        self.reset_publisher.publish(Empty())
        self._last_total = 0
        if self.gui_history_cb:
            ts = datetime.now().strftime('%Y/%m/%d %H:%M:%S')
            self.gui_history_cb(f'{ts}  Reset')

# ────────────────────────────  GUI ────────────────────────────
class MoneyGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Money Counter")

        # Canvas + Scrollbar
        self.canvas = tk.Canvas(self.root, highlightthickness=0)
        self.vsb = tk.Scrollbar(self.root, orient="vertical",
                                command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.vsb.set)
        self.canvas.pack(fill="both", expand=True, side="left")
        self.vsb.pack_forget()                          # 先隱藏捲軸
        self.root.bind("<Configure>", self._on_resize)
        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)
        self.canvas.bind_all("<Button-4>", self._on_mousewheel)
        self.canvas.bind_all("<Button-5>", self._on_mousewheel)

        # state
        self.current_value = 0
        self.page = 'main'
        self.history_lines = []
        self.node = None
        self.items = {}

        # fonts
        self.title_font = tkFont.Font(family="Montserrat", weight="bold")
        self.date_font  = tkFont.Font(family="Montserrat")
        self.hist_font  = tkFont.Font(family="Consolas")
        self.hist_scale = 0.015

        # buttons
        self.reset_btn   = tk.Button(self.root, text="Reset",   command=self._on_reset)
        self.history_btn = tk.Button(self.root, text="History", command=self._show_history)
        self.back_btn    = tk.Button(self.root, text="Back",    command=self._show_main)
        self.clean_btn   = tk.Button(self.root, text="Clean",   command=self._clear_history)
        self.reset_btn_win = self.history_btn_win = None
        self.back_btn_win = self.clean_btn_win   = None

        self._draw_all()

    # public API
    def set_node(self, node): self.node = node
    def update_current(self, v): self.current_value = v; \
        self.canvas.itemconfigure(self.items.get('current',''), text=f"${v:,.2f}")
    def add_history(self, t, keep_last=500):
        self.history_lines.append(t)
        if len(self.history_lines) > keep_last:
            self.history_lines = self.history_lines[-keep_last:]
        if self.page == 'history': self._draw_all()

    # events
    def _on_reset(self):
        self.update_current(0)
        if self.node: self.node.publish_reset()
    def _show_history(self):
        self.page='history'; self._draw_all()
    def _show_main(self):
        self.page='main'; self._draw_all()
    def _clear_history(self):
        self.history_lines.clear(); self._draw_all()
    def _on_mousewheel(self,event):
        if self.page!='history': return
        delta = -1 if (event.num==5 or event.delta<0) else 1
        self.canvas.yview_scroll(delta, "units")
    def _on_resize(self,e):
        if e.widget==self.root: self._draw_all(e.width,e.height)

    # drawing
    def _draw_all(self,w=None,h=None):
        w = w or self.canvas.winfo_width() or self.root.winfo_width()
        h = h or self.canvas.winfo_height() or self.root.winfo_height()
        self.canvas.delete("all")
        self.canvas.yview_moveto(0)          # 回到頂端

        # show/hide scrollbar
        if self.page=='history': self.vsb.pack(fill="y",side="right")
        else:                     self.vsb.pack_forget()

        self.title_font.configure(size=int(h*0.10))
        self.date_font.configure(size=int(h*0.02))
        self.hist_font.configure(size=int(h*self.hist_scale))

        # decide bg height
        if self.page=='history':
            y_start = h*0.18
            line_h  = self.hist_font.cget('size')*6.0
            content_h = y_start + line_h*len(self.history_lines)
            bg_h = max(h, content_h + h*0.2)     # 畫到內容底再多一點
        else:
            bg_h = h

        self._draw_gradient("#1a1a3d", "#2e2a3d", w, bg_h, steps=120)

        if self.page=='main':
            self.items['current']=self.canvas.create_text(
                w/2,h*0.30,text=f"${self.current_value:,.2f}",
                font=self.title_font,fill="white")
            self.items['date']=self.canvas.create_text(
                w/2,h*0.46,text=datetime.now().strftime('%d %B %Y'),
                font=self.date_font,fill="white")
            if self.history_btn_win: self.canvas.delete(self.history_btn_win)
            if self.reset_btn_win:   self.canvas.delete(self.reset_btn_win)
            self.history_btn_win=self.canvas.create_window(w/2,h*0.65,window=self.history_btn)
            self.reset_btn_win  =self.canvas.create_window(w/2,h*0.75,window=self.reset_btn)

        else:   # history page
            if self.back_btn_win:  self.canvas.delete(self.back_btn_win)
            if self.clean_btn_win: self.canvas.delete(self.clean_btn_win)
            self.back_btn_win = self.canvas.create_window(w*0.10,h*0.08,window=self.back_btn)
            self.clean_btn_win= self.canvas.create_window(w*0.90,h*0.08,window=self.clean_btn)

            for i,txt in enumerate(reversed(self.history_lines)):
                y = y_start + i*line_h
                self.canvas.create_text(w*0.05,y,anchor='nw',
                    text=txt,font=self.hist_font,fill="white")
                self.canvas.create_line(w*0.05,y+line_h*0.8,
                    w*0.95,y+line_h*0.8,fill="#5c5c5c",width=1)

            self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    # gradient helper
    def _draw_gradient(self,c1,c2,w,h,steps=100):
        r1,g1,b1=self.root.winfo_rgb(c1); r2,g2,b2=self.root.winfo_rgb(c2)
        dr,dg,db=(r2-r1)/steps,(g2-g1)/steps,(b2-b1)/steps
        for i in range(steps):
            nr=int(r1+dr*i); ng=int(g1+dg*i); nb=int(b1+db*i)
            color=f"#{nr>>8:02x}{ng>>8:02x}{nb>>8:02x}"
            y1=int(h*i/steps); y2=int(h*(i+1)/steps)
            self.canvas.create_rectangle(0,y1,w,y2,outline="",fill=color)

    def run(self): self.root.mainloop()

# ────────────────────────────  main ────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    gui = MoneyGUI()
    node = MoneyCountNode(gui_update_cb=gui.update_current,
                          gui_history_cb=gui.add_history)
    gui.set_node(node)
    Thread(target=lambda: (rclpy.spin(node), node.destroy_node(), rclpy.shutdown()),
           daemon=True).start()
    gui.run()

if __name__ == '__main__':
    main()
