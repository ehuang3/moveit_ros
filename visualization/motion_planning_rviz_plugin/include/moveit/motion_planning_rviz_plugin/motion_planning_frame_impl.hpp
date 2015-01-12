#pragma once

template<typename T>
void MotionPlanningFrame::waitForAction(const T &action, const ros::NodeHandle &node_handle,
                                        const ros::Duration &wait_for_server, const std::string &name)
{
  ROS_DEBUG("Waiting for MoveGroup action server (%s)...", name.c_str());

  // in case ROS time is published, wait for the time data to arrive
  ros::Time start_time = ros::Time::now();
  while (start_time == ros::Time::now())
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
  }

  // wait for the server (and spin as needed)
  if (wait_for_server == ros::Duration(0, 0))
  {
    // wait forever until action server connects
    while (node_handle.ok() && !action->isServerConnected())
    {
      ros::WallDuration(0.02).sleep();
      ros::spinOnce();
    }
  }
  else
  {
    // wait for a limited amount of non-simulated time
    ros::WallTime final_time = ros::WallTime::now() + ros::WallDuration(wait_for_server.toSec());
    while (node_handle.ok() && !action->isServerConnected() && final_time > ros::WallTime::now())
    {
      ros::WallDuration(0.02).sleep();
      ros::spinOnce();
    }
  }

  if (!action->isServerConnected())
    throw std::runtime_error("Unable to connect to move_group action server within allotted time");
  else
    ROS_DEBUG("Connected to '%s'", name.c_str());
};

template< typename Message > QByteArray MotionPlanningFrame::serializeMessage(const Message& msg)
{
    // Stored data.
    QByteArray string;

    // Get the length of serialization.
    uint32_t serial_size = ros::serialization::serializationLength(msg);

    // Construct a middle man for the serialization?
    boost::scoped_array<uint8_t> buffer(new uint8_t[serial_size]);

    // Create a serialization object.
    ros::serialization::OStream stream(buffer.get(), serial_size);

    // Serialize.
    ros::serialization::serialize(stream, msg);

    // Clear QByteArray.
    string.clear();

    // Copy to QByteArray.
    for (uint32_t i = 0; i < serial_size; i++)
    {
        char b = (char) buffer.get()[i];
        string.append(b);
    }

    return string;
};

template< typename Message > Message MotionPlanningFrame::deserializeMessage(const QByteArray& string)
{
    // Deserialized message.
    Message msg;

    // Get the length of serialization.
    uint32_t serial_size = (uint32_t) (string.size() / sizeof(char));

    // Construct a middle man for the deserialization?
    boost::scoped_array<uint8_t> buffer(new uint8_t[serial_size]);

    // Copy to buffer.
    for (uint32_t i = 0; i < serial_size; i++)
    {
        buffer.get()[i] = (uint8_t) string.at(i);
    }

    // Create a deserialization object.
    ros::serialization::IStream stream(buffer.get(), serial_size);

    // Deserialize.
    ros::serialization::Serializer<Message>::read(stream, msg);

    return msg;
};

template< typename Message > Message MotionPlanningFrame::getMessageFromUserData(const QVariant& data)
{
    // The message we are getting.
    Message msg;

    // Check if the data can be convertered.
    if (!data.canConvert(QVariant::Map))
    {
        ROS_ERROR("Failed to convert stored data to QVariantMap.");
        return msg;
    }

    QVariantMap map = data.toMap();

    // Auto-generate key for message type. May not be human readable.
    QString key = "DUMB_KEY"; // typeid(Message).name();

    if (!map.contains(key))
    {
        ROS_ERROR("Stored data does not contain %s.", key.toStdString().c_str());
        return msg;
    }

    QVariant msg_data = map[key];

    // Check if the data can be convertered.
    if (!msg_data.canConvert(QVariant::ByteArray))
    {
        ROS_ERROR("Failed to convert stored data to QByteArray.");
        return msg;
    }

    // Convert the variant to a byte array.
    QByteArray byte_array = msg_data.toByteArray();

    // Deserialize message.
    msg = deserializeMessage<Message>(byte_array);

    return msg;
};

template< typename Message > void MotionPlanningFrame::setMessageToUserData(QVariant& data, const Message& msg)
{
    // Ensure the data is a QVariantMap.
    if (!data.canConvert(QVariant::Map))
        data = QVariantMap();

    // Get the underlying map.
    QVariantMap map = data.toMap();

    // Create a byte array.
    QByteArray byte_array;

    // Serialize the goal message into a byte array.
    byte_array = serializeMessage<Message>(msg);

    // Auto-generate key for message type. May not be human readable.
    QString key = "DUMB_KEY"; // typeid(Message).name();

    // Set the goal message into the map.
    map[key] = byte_array;

    // Set the map back into the data.
    data = map;
};
