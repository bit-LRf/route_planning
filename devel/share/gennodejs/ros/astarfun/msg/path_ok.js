// Auto-generated. Do not edit!

// (in-package astarfun.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class path_ok {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.path_ok = null;
    }
    else {
      if (initObj.hasOwnProperty('path_ok')) {
        this.path_ok = initObj.path_ok
      }
      else {
        this.path_ok = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type path_ok
    // Serialize message field [path_ok]
    bufferOffset = _serializer.bool(obj.path_ok, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type path_ok
    let len;
    let data = new path_ok(null);
    // Deserialize message field [path_ok]
    data.path_ok = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'astarfun/path_ok';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '276f6ed1d160276c34d8bfda3a611a63';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool path_ok
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new path_ok(null);
    if (msg.path_ok !== undefined) {
      resolved.path_ok = msg.path_ok;
    }
    else {
      resolved.path_ok = false
    }

    return resolved;
    }
};

module.exports = path_ok;
