# Full time-parameterized trajectory, published once per (re)plan, latched.
# Consumers index into samples by (now - header.stamp) / dt.
std_msgs/Header header        # stamp = trajectory start time t0
string name                   # trajectory key (e.g. for per-trajectory learning)
float64 dt                    # sample spacing [s]
ReferenceTraj[] samples
