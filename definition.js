Blockly.Blocks['veml6040_sensor_read_color'] = {
  init: function() {
    this.jsonInit({
      "type": "veml6040_sensor_read_color",
      "message0": "cảm biến màu sắc đọc giá trị %1",
      "args0": [
        {
          "type": "field_dropdown",
          "name": "COLOR",
          "options": [
            ["độ rọi màu (lux)", "LUX"],  // Đổi thành "độ rọi màu (lux)"
            ["đỏ", "RED"],
            ["xanh lá", "GREEN"],
            ["xanh dương", "BLUE"],
            ["nhiệt độ màu", "CCT"]
          ]
        }
      ],
      "output": "Number",
      "colour": "#ae00ae",
      "tooltip": "Đọc giá trị màu sắc RGB từ cảm biến",
      "helpUrl": ""
    });
  }
};

Blockly.Python['veml6040_sensor_read_color'] = function(block) {
  var color = block.getFieldValue('COLOR');
  var code = '';

  if (color === 'LUX') {
    code = 'veml6040_sensor.get_lux()';
  } else if (color === 'CCT') {
    code = 'veml6040_sensor.get_cct()';
  } else {
    code = 'veml6040_sensor.get_' + color.toLowerCase() + '()';
  }

  Blockly.Python.definitions_['import_veml6040_sensor'] = 'from veml6040_sensor import *';
  
  return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Blocks['veml6040_sensor_detect_color'] = {
  init: function() {
    this.jsonInit({
      "type": "veml6040_sensor_detect_color",
      "message0": "cảm biến màu sắc phát hiện màu %1",
      "args0": [
        {
          "type": "field_dropdown",
          "name": "DETECT_COLOR",
          "options": [
            ["trắng", "white"],
            ["đen", "black"],
            ["đỏ", "red"],
            ["xanh lá", "green"],
            ["xanh dương", "blue"],
            ["vàng", "yellow"]
          ]
        }
      ],
      "output": "Boolean",
      "colour": "#ae00ae",
      "tooltip": "Phát hiện màu sắc cụ thể",
      "helpUrl": ""
    });
  }
};

Blockly.Python['veml6040_sensor_detect_color'] = function(block) {
  var detectColor = block.getFieldValue('DETECT_COLOR');
  var code = '(veml6040_sensor.classify_hue(hues={"red":0,"yellow":60,"green":120,"cyan":180,"blue":240,"magenta":300}) == "' + detectColor + '")';
  return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Blocks['veml6040_sensor_read_lux'] = {
  init: function() {
    this.jsonInit({
      "type": "veml6040_sensor_read_lux",
      "message0": "cảm biến màu sắc đọc giá trị độ rọi màu (lux)", 
      "output": "Number",
      "colour": "#ae00ae",
      "tooltip": "Đọc giá trị độ rọi màu lux",
      "helpUrl": ""
    });
  }
};

Blockly.Python['veml6040_sensor_read_lux'] = function(block) {
  var code = 'veml6040_sensor.get_lux()'; 
  return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Blocks['veml6040_sensor_read_cct'] = {
  init: function() {
    this.jsonInit({
      "type": "veml6040_sensor_read_cct",
      "message0": "cảm biến màu sắc đọc giá trị nhiệt độ màu",
      "output": "Number",
      "colour": "#ae00ae",
      "tooltip": "Đọc giá trị nhiệt độ màu (CCT)",
      "helpUrl": ""
    });
  }
};

Blockly.Python['veml6040_sensor_read_cct'] = function(block) {
  var code = 'veml6040_sensor.get_cct()'; 
  return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.definitions_['import_veml6040_sensor'] = 'from veml6040_sensor import ColorSensorVEML6040';
