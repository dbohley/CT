import pandas as pd
import plotly.express as px

# 1. Load the data
name = "sara"
df = pd.read_csv(rf"breathe_profiles\{name}_normal_breathing.csv")

# 2. Create an interactive line chart
fig = px.line(
    df, 
    x="time_s", 
    y="y_mm", 
    title=f"{name}'s Displacement vs Time (Normal Breathing)",
    labels={"time_s": "Time (s)", "y_mm": "Displacement (mm)"}
)

# 3. Add a scrollable range slider to the x-axis
fig.update_xaxes(rangeslider_visible=True)

# 4. Display the plot (this will open in your default web browser)
fig.show()