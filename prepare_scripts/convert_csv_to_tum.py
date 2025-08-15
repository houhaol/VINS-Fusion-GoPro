import pandas as pd

# Read the CSV file, skip the header row
df = pd.read_csv("trajectory.csv", header=0)

# Prepare output DataFrame in TUM format:
# timestamp[s] tx ty tz qx qy qz qw
out = pd.DataFrame({
	0: df.iloc[:, 0] / 1e9,     # timestamp (ns to s)
	1: df.iloc[:, 1],           # tx
	2: df.iloc[:, 2],           # ty
	3: df.iloc[:, 3],           # tz
	4: df.iloc[:, 5],           # qx
	5: df.iloc[:, 6],           # qy
	6: df.iloc[:, 7],           # qz
	7: df.iloc[:, 4],           # qw
})

out.to_csv("trajectory_tum.txt", sep=" ", header=False, index=False, float_format="%.9f")
