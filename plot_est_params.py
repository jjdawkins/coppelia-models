import rosbag2_py
import matplotlib.pyplot as plt

# Path to the rosbag2 file
bag_file = "/home/ew-admin/coppelia-models/rosbag2_2024_08_14-13_16_33/rosbag2_2024_08_14-13_16_33_0.db3"

# Topic to read from
topic = "/rover/est_param"

# Lists to store the data
timestamps = []
data = []

# Create a reader
reader = rosbag2_py.SequentialReader()

# Set the storage options and open the bag file
storage_options = rosbag2_py.StorageOptions(uri=bag_file, storage_id="sqlite3")
converter_options = rosbag2_py.ConverterOptions(
    input_serialization_format="cdr", output_serialization_format="cdr"
)
reader.open(storage_options, converter_options)

# Get metadata about the bag
metadata = reader.get_metadata()
topics_and_types = metadata.topics_with_message_count

# Check if the topic exists in the bag
if topic not in [t.name for t in topics_and_types]:
    print(f"Topic {topic} not found in the bag file.")
else:
    # Read messages from the specified topic
    while reader.has_next():
        (topic_name, data, t) = reader.read_next()
        if topic_name == topic:
            # Assuming the message is of type Float32MultiArray
            # Modify this part if the message type is different
            msg = rosbag2_py.deserialize_message(data, "std_msgs/msg/Float32MultiArray")
            if msg.data:
                timestamps.append(msg.data[0])
                data.append(msg.data[1:])  # All elements except the 0th element

# Convert data to a format suitable for plotting
data = list(zip(*data))  # Transpose the list of lists

# Plot the data
for i, y_data in enumerate(data):
    plt.plot(timestamps, y_data, label=f"Element {i+1}")

plt.xlabel("Time")
plt.ylabel("Data")
plt.title("Plot of Float32MultiArray")
plt.legend()
plt.show()
