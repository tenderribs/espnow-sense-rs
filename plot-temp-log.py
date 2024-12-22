import pandas as pd
from matplotlib import pyplot as plt

# Read the CSV file
df = pd.read_csv("LOG.CSV", sep=",", header=None, names=["timestamp", "value"])

# Convert the 'timestamp' column to datetime format (optional, if applicable)
df["timestamp"] = pd.to_datetime(df["timestamp"])

# Plot the data
plt.plot(df["timestamp"], df["value"])
plt.xlabel("Timestamp")
plt.ylabel("Value")
plt.title("Log Data Plot")

plt.grid(True)
plt.show()
