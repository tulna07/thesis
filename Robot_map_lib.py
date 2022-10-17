win_size = 100

class Map:
    
    def generate(self, plt, i, part, MAX_VERTICES):
        # displaying the title 
        plt.title("Obstacle part[{0}/{1}]: Click on plot to generate data\n".format(i, part) +
            " Middle click to turn next obstacle, MAX_VERTICES={0}".format(MAX_VERTICES))
        plt.axis([0, win_size, 0, win_size])

        return plt.ginput(MAX_VERTICES, show_clicks=True, timeout=-1)  # no timeout

    def display(self, plt, title, obstacles, alpha: float = 0.4, hatch: str = '//////'):
        
        # displaying the title 
        plt.title(title)
        for obstacle in obstacles:
            x = [point[0] for point in obstacle]
            y = [point[1] for point in obstacle]
            x.append(obstacle[0][0])
            y.append(obstacle[0][1])
            plt.fill(x, y, color='k', alpha=alpha, hatch=hatch)
