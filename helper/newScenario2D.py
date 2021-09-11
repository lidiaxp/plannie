import tkinter as tk

class Cenario:
    def __init__(self):
        self.click = 0
        self.auxX = []
        self.auxY = []
        self.pathX = []
        self.pathY = []
        self.visX = []
        self.visY = []
        
tamTotal = 10
espacamento = 0.2
n = int((tamTotal/espacamento) + 1)

start = 15
tamCircle = 5
space = 20

master = tk.Tk()
master.title("Create 2D Environment")

w = tk.Canvas(master, width=(tamTotal*100)+(start*2), height=(tamTotal*100)+(start*2))
w.pack()

c = Cenario()

for j in range(n):
    for i in range(n):
        color = "black" if i % 5 == 0 and j % 5 == 0 else "yellow"
        w.create_oval(start+(space*i), start+(space*j), (start+tamCircle)+(space*i), (start+tamCircle)+(space*j), fill=color, tags='oval')
    
def defineXY(event):
    x, y = event.x, event.y
    ivx = (((x - (start+tamCircle)) / space) * espacamento) * 10
    ivy = (((y - start) / space) * espacamento) * 10
    
    if round(ivx) % 2 == 1:
        fvx = (round(ivx) - 1)/10.0 if ivx <= round(ivx) else (round(ivx)+1)/10.0
    else:
        fvx = round(ivx)/10.0

    if round(ivy) % 2 == 1:
        fvy = (round(ivy) - 1)/10.0 if ivy <= round(ivy) else (round(ivy)+1)/10.0
    else:
        fvy = round(ivy)/10.0
    fvy = tamTotal - fvy

    return fvx, fvy

# def limparLinha(event):
#     c.click = 0
#     print("Apagou essa Linha")

# def apagarTudo(event):
#     c.auxX = []
#     c.auxY = []
#     c.pathX = []
#     c.pathY = []
#     print("Apagou Tudo")

def export(event):
    print("Path X")
    print(c.pathX)
    print("Path Y")
    print(c.pathY)

def createLine(event):    
    c.click += 1
    x, y = defineXY(event)
    t = 1
    if c.click%2==0:
        c.pathX.append(c.auxX[0])
        c.pathY.append(c.auxY[0])
        c.visX.append(c.auxX[1])
        c.visY.append(c.auxY[1])
        c.pathX.append(x)
        c.pathY.append(y)
        c.visX.append(event.x)
        c.visY.append(event.y)
        c.click, c.auxY, c.auxX = 0, 0, 0
        w.create_oval(event.x-t, event.y-t, event.x+t, event.y+t, fill="red")
    else:
        c.auxX = [x, event.x]
        c.auxY = [y, event.y]
        w.create_oval(event.x-t, event.y-t, event.x+t, event.y+t, fill="red")

    if len(c.visX)%2 == 0 and len(c.visX) > 0:
        for i in range(len(c.visX)):
            if i%2==0:
                sx, sy = c.visX[i], c.visY[i]
            else:
                w.create_line(sx, sy, c.visX[i], c.visY[i], fill="red")
                sx, sy = 0, 0

w.tag_bind('oval','<Button>',func=createLine)
w.bind("<Leave>", export)
# w.bind("<Button-3>", limparLinha)
# w.bind("<Double-Button-3>", apagarTudo)

tk.mainloop()