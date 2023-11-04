import ROSLIB from "roslib";
import { useEffect, useState } from "react";
import { slider_name } from "../static/Vars";
import { empty } from "../assets";

export const Field = () => {
  const [sliderVal, setSliderVal] = useState([0, 0, 0, 0, 0, 0]);
  const [connectionStat, setConnectionStat] = useState("Disconnected");
  const [imageUrl, setImageUrl] = useState(empty);
  const [ros, setRos] = useState(null);

  //   const newRosConn = new ROSLIB.Ros({
  //     url: "ws://192.168.10.158:9900",
  //   });

  //   newRosConn.on(`connection`, () => {
  //     setConnectionStat("Connected");
  //   });

  useEffect(() => {
    const newRosConn = new ROSLIB.Ros({
      //   url: "ws://192.168.10.158:9900",
      url: `ws://${window.location.hostname}:9900`,
    });

    newRosConn.on("connection", () => {
      setConnectionStat("Connected");
      console.log("Connected to websocket server.");
    });

    newRosConn.on("error", (error) => {
      console.log("Error connecting to websocket server: ", error);
    });

    newRosConn.on("close", () => {
      setConnectionStat("Disconnected");
    });

    // Setup the publisher on the '/vision/field/threshold/params' topic
    const paramsPublisher = new ROSLIB.Topic({
      ros: newRosConn,
      name: "/vision/field/threshold/params",
      messageType: "std_msgs/UInt8MultiArray",
    });

    const sliderService = new ROSLIB.Service({
      ros: newRosConn,
      name: "/vision/field/threshold/params/srv",
      serviceType: "msg_collection/ThresholdVision",
    });

    const sliderRequest = new ROSLIB.ServiceRequest({});

    sliderService.callService(sliderRequest, (result) => {
      console.log("Result for service call on " + sliderService.name + ": ");
      console.log(result);
      for (let i = 0; i < sliderVal.length; i++) {
        setSliderVal((prevState) => {
          const newState = [...prevState];
          newState[i] = result.threshold_params[i];
          return newState;
        });
      }
    });

    // Save ROS publisher and service to state
    setRos({
      ros: newRosConn,
      publisher: paramsPublisher,
      service: sliderService,
    });

    // Clean up on unmount
    return () => {
      newRosConn.close();
    };
  }, []);

  const handleSliderChange = (index, value) => {
    setSliderVal((prevState) => {
      const newState = [...prevState];
      newState[index] = value;
      return newState;
    });
  };

  useEffect(() => {
    if (ros && ros.publisher) {
      const message = new ROSLIB.Message({
        data: sliderVal,
      });
      ros.publisher.publish(message);
    }
  }, [sliderVal, ros]);

  return (
    <>
      <div className="flex justify-center items-center">
        <p className="text-xl">Field {connectionStat}</p>
      </div>
      <div className="w-full max-h-screen grid grid-cols-4 gap-x-10 px-10 ">
        <div id="image_placeholder_1" className="w-fit h-fit">
          <img
            src={`http://${window.location.hostname}:9901/stream?topic=/vision/raw/frame`}
            alt=""
          />
        </div>
        <div id="image_placeholder_2" className="w-fit h-fit">
          <img
            src={`http://${window.location.hostname}:9901/stream?topic=/vision/display/frame`}
            alt=""
          />
        </div>
        <div id="image_placeholder_2" className="w-fit h-fit">
          <img
            src={`http://${window.location.hostname}:9901/stream?topic=/vision/thresholded/field`}
            alt=""
          />
        </div>
        <div
          id="image_placeholder_3"
          className="w-full h-fit flex flex-col my-auto"
        >
          {[...Array(6).keys()].map((index) => (
            <div key={index} className="">
              <label
                htmlFor={`slider-${index}`}
                className="block mb-2 text-sm font-medium text-gray-900"
              >
                {slider_name[index]} : {sliderVal[index]}
              </label>
              <input
                id={`slider-${index}`}
                type="range"
                min={0}
                max={index === 0 || index === 1 ? 128 : 255}
                step={1}
                value={sliderVal[index]}
                onChange={(e) =>
                  handleSliderChange(index, parseInt(e.target.value))
                }
                className="w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer"
              />
            </div>
          ))}
        </div>
      </div>
    </>
  );
};
