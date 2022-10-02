import numpy as np
import matplotlib.pyplot as plt

show_animation = True


class BugPlanner:
    def __init__(self, start_x, start_y, goal_x, goal_y, obs_x, obs_y):
        #initalizing the varaibles
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.obs_x = obs_x
        self.obs_y = obs_y
        self.r_x = [start_x]
        self.r_y = [start_y]
        self.out_x = []
        self.out_y = []
        #aggregating the both varaibles
        for o_x, o_y in zip(obs_x, obs_y):
            for add_x, add_y in zip([1, 0, -1, -1, -1, 0, 1, 1],
                                    [1, 1, 1, 0, -1, -1, -1, 0]):
                cand_x, cand_y = o_x+add_x, o_y+add_y
                valid_point = True
                for _x, _y in zip(obs_x, obs_y):
                    if cand_x == _x and cand_y == _y:
                        valid_point = False
                        break
                if valid_point:
                    self.out_x.append(cand_x), self.out_y.append(cand_y)

    def mov_normal(self):
        # It moves the robot normally
        return self.r_x[-1] + np.sign(self.goal_x - self.r_x[-1]), \
               self.r_y[-1] + np.sign(self.goal_y - self.r_y[-1])

    def mov_to_next_obs(self, visited_x, visited_y):
        # It executes when we move to next obstacle
        for add_x, add_y in zip([1, 0, -1, 0], [0, 1, 0, -1]):
            c_x, c_y = self.r_x[-1] + add_x, self.r_y[-1] + add_y
            for _x, _y in zip(self.out_x, self.out_y):
                use_pt = True
                if c_x == _x and c_y == _y:
                    for v_x, v_y in zip(visited_x, visited_y):
                        if c_x == v_x and c_y == v_y:
                            use_pt = False
                            break
                    if use_pt:
                        return c_x, c_y, False
                if not use_pt:
                    break
        return self.r_x[-1], self.r_y[-1], True

    def bug2(self):
        
        mov_dir = 'normal'
        cand_x, cand_y = -np.inf, -np.inf
        if show_animation:
            #It helps to show graphical representation of the given data in X and Y axis
            plt.plot(self.obs_x, self.obs_y, ".k")
            plt.plot(self.r_x[-1], self.r_y[-1], "og")
            plt.plot(self.goal_x, self.goal_y, "xb")
            plt.plot(self.out_x, self.out_y, ".")

        straight_x, straight_y = [self.r_x[-1]], [self.r_y[-1]]
        hit_x, hit_y = [], []
        while True:
            if straight_x[-1] == self.goal_x and \
                    straight_y[-1] == self.goal_y:
                break
            c_x = straight_x[-1] + np.sign(self.goal_x - straight_x[-1]) #It useful and provide sign of a given number
            c_y = straight_y[-1] + np.sign(self.goal_y - straight_y[-1])
            for x_ob, y_ob in zip(self.out_x, self.out_y):
                if c_x == x_ob and c_y == y_ob:
                    hit_x.append(c_x), hit_y.append(c_y)     #append means adding the values of c_x variable to hit_x
                    break
            straight_x.append(c_x), straight_y.append(c_y)
        if show_animation:
            #plotting the data 
            plt.plot(straight_x, straight_y, ",")
            plt.plot(hit_x, hit_y, "d")
            plt.grid(True)
            plt.title('BUG 2')

        for x_ob, y_ob in zip(self.out_x, self.out_y):
            if self.r_x[-1] == x_ob and self.r_y[-1] == y_ob:
                mov_dir = 'obs'
                break

        visited_x, visited_y = [], []
        while True:
            if self.r_x[-1] == self.goal_x \
                    and self.r_y[-1] == self.goal_y:
                break
            if mov_dir == 'normal':
                cand_x, cand_y = self.mov_normal()
            if mov_dir == 'obs':
                cand_x, cand_y, _ = self.mov_to_next_obs(visited_x, visited_y)
            if mov_dir == 'normal':
                found_boundary = False
                for x_ob, y_ob in zip(self.out_x, self.out_y):
                    if cand_x == x_ob and cand_y == y_ob:
                        self.r_x.append(cand_x), self.r_y.append(cand_y)
                        visited_x[:], visited_y[:] = [], []
                        visited_x.append(cand_x), visited_y.append(cand_y)
                        del hit_x[0]
                        del hit_y[0]
                        mov_dir = 'obs'
                        found_boundary = True
                        break
                if not found_boundary:
                    self.r_x.append(cand_x), self.r_y.append(cand_y)
            elif mov_dir == 'obs':
                self.r_x.append(cand_x), self.r_y.append(cand_y)
                visited_x.append(cand_x), visited_y.append(cand_y)
                # Zip is an iterator that will aggregate elements from hit_x and hit_y 
                for i_x, i_y in zip(range(len(hit_x)), range(len(hit_y))):
                    if cand_x == hit_x[i_x] and cand_y == hit_y[i_y]:
                        del hit_x[i_x]
                        del hit_y[i_y]
                        mov_dir = 'normal'
                        break
            if show_animation:
                plt.plot(self.r_x, self.r_y, "-r")
                plt.pause(0.001)
        if show_animation:
            #It is useful to display all the current active figure object and display the figures
            plt.show()

def main(bug_2):
    o_x, o_y = [], []
                                    
    s_x = 0.0                #Assigning the X and Y axis values of Robot Position  
    s_y = 50.0
    g_x = 180.0                #Assigning the X and Y axis values of Goal where Robot should reach
    g_y = 50.0

   #Below are the loops used for positioning the obstacle
    for i in range(60, 100):
        for j in range(40, 80):           
            o_x.append(i)
            o_y.append(j)
              
    for i in range(120, 140):
        for j in range(80, 100):
            o_x.append(i)
            o_y.append(j)

    for i in range(120, 160):
        for j in range(40, 60):
            o_x.append(i)
            o_y.append(j)
    
    if bug_2:
        # calling the BugPlanner Class and storing the result in my_Bug
        my_Bug = BugPlanner(s_x, s_y, g_x, g_y, o_x, o_y)
        my_Bug.bug2()


if __name__ == '__main__':
    main(bug_2=True)
