import matplotlib.pyplot as plt
import matplotlib.animation as animation 

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

def animate(i):
    data = open('stock.txt', 'r').read()
    lines = data.split('\n')
    xs = []
    ys = []

    for line in lines: 
        y,x = line.split(',') # la comma es el delimitador
        xs.append(float(x))
        ys.append(float(y))
    ax1.clear()
    ax1.plot(xs,ys,'--', marker="o", color = "cornflowerblue")

    plt.xlabel('Longitud')
    plt.ylabel('Latitud')
    plt.title('Localización ...')
    plt.xticks(fontsize=8)
    plt.grid(True)

ani = animation.FuncAnimation(fig,animate,interval=1000)
plt.show()