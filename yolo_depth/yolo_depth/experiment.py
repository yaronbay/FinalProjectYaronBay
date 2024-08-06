import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

# Function to generate random data for the histogram
def generate_random_data():
    return np.random.randint(0, 100, size=100)

# Function to update the histogram
def update(frame):
    data = generate_random_data()  # Generate random data
    ax.clear()
    ax.hist(data, bins=10, edgecolor='black')
    ax.set_xlabel('Value')
    ax.set_ylabel('Frequency')
    ax.set_title('Histogram')

# Create a figure and axis
fig, ax = plt.subplots()

# Create the animation
ani = FuncAnimation(fig, update, interval=3000)  # Update every 3 seconds

plt.show()