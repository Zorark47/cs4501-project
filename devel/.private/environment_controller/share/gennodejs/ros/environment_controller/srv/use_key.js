// Auto-generated. Do not edit!

// (in-package environment_controller.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class use_keyRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.door_loc = null;
    }
    else {
      if (initObj.hasOwnProperty('door_loc')) {
        this.door_loc = initObj.door_loc
      }
      else {
        this.door_loc = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type use_keyRequest
    // Serialize message field [door_loc]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.door_loc, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type use_keyRequest
    let len;
    let data = new use_keyRequest(null);
    // Deserialize message field [door_loc]
    data.door_loc = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'environment_controller/use_keyRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3ef7d5d942900fab8f329a5d05ac1650';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point door_loc
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new use_keyRequest(null);
    if (msg.door_loc !== undefined) {
      resolved.door_loc = geometry_msgs.msg.Point.Resolve(msg.door_loc)
    }
    else {
      resolved.door_loc = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

class use_keyResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type use_keyResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type use_keyResponse
    let len;
    let data = new use_keyResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'environment_controller/use_keyResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new use_keyResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: use_keyRequest,
  Response: use_keyResponse,
  md5sum() { return '31c1bb88ea2fc30c8bbcc2144b52d6f7'; },
  datatype() { return 'environment_controller/use_key'; }
};
