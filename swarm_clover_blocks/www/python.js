/*
 * Copyright (C) 2020 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

import {params} from './ros.js';

// If any new block imports any library, add that library name here.
Blockly.Python.addReservedWords('_b,_print');
Blockly.Python.addReservedWords('rospy,srv,Trigger,get_telemetry,navigate,set_velocity,land');
Blockly.Python.addReservedWords('navigate_wait,land_wait,wait_arrival,wait_yaw,get_distance');
// Blockly.Python.addReservedWords('pigpio,pi,Range');
Blockly.Python.addReservedWords('SetLEDEffect,set_effect,led_count,get_led_count');
Blockly.Python.addReservedWords('SetLEDs,LEDState,set_leds');

const IMPORT_SRV = `from clover import srv
from std_srvs.srv import Trigger`;

function NAVIGATE_WAIT(id) { 
	return `
	\ndef navigate_wait_${id}(x=0, y=0, z=0, speed=0.5, frame_id='body${id}', auto_arm=False):
    res = navigate_${id}(x=x, y=y, z=z, yaw=float('nan'), speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    if not res.success:
        raise Exception(res.message)
    while not rospy.is_shutdown():
        telem = get_telemetry_${id}(frame_id='navigate_target${id}')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < ${params.navigate_tolerance}:
            return
        rospy.sleep(${params.sleep_time})\n`;
	}

function LAND_WAIT(id) {
	return `\ndef land_wait_${id}():
    land_${id}()
    while get_telemetry_${id}().armed:
        rospy.sleep(${params.sleep_time})\n`;
	}

function WAIT_YAW(id) { 
	return `\ndef wait_yaw_${id}():
    while not rospy.is_shutdown():
        telem = get_telemetry_${id}(frame_id='navigate_target${id}')
        if abs(telem.yaw) < math.radians(${params.yaw_tolerance}):
            return
        rospy.sleep(${params.sleep_time})\n`;
	}

function WAIT_ARRIVAL(id) {
	return `\ndef wait_arrival_${id}():
    while not rospy.is_shutdown():
        telem = get_telemetry_${id}(frame_id='navigate_target${id}')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < ${params.navigate_tolerance}:
            return
        rospy.sleep(${params.sleep_time})\n`;
	}

function ARRIVED(id) { 
	return `\ndef arrived():
    telem = get_telemetry_${id}(frame_id='navigate_target${id}')
    return math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < ${params.navigate_tolerance}\n`
	}

function GET_DISTANCE(id) {
	return `\ndef get_distance_${id}(x, y, z, frame_id):
    telem = get_telemetry_${id}(frame_id)
    return math.sqrt((x - telem.x) ** 2 + (y - telem.y) ** 2 + (z - telem.z) ** 2)\n`;
}

var rosDefinitions = {};
function generateROSDefinitions(id) {
	
	// order for ROS definitions is significant, so generate all ROS definitions as one
	Blockly.Python.definitions_[`node`] = `rospy.init_node('flight')\n`;
	var code = `# Defining clover${id} ROS services\n`;
	if (rosDefinitions.offboard) {
		code += `get_telemetry_${id} = rospy.ServiceProxy('clover${id}/get_telemetry', srv.GetTelemetry)\n`;
		code += `navigate_${id} = rospy.ServiceProxy('clover${id}/navigate', srv.Navigate)\n`;
		if (rosDefinitions.setVelocity) {
			code += `set_velocity_${id} = rospy.ServiceProxy('clover${id}/set_velocity', srv.SetVelocity)\n`;
		}
		if (rosDefinitions.setAttitude) {
			code += `set_attitude_${id} = rospy.ServiceProxy('clover${id}/set_attitude', srv.SetAttitude)\n`;
		}
		if (rosDefinitions.setRates) {
			code += `set_rates_${id} = rospy.ServiceProxy('clover${id}/set_rates', srv.SetRates)\n`;
		}
		code += `land_${id} = rospy.ServiceProxy('clover${id}/land', Trigger)\n`;
	}
	if (rosDefinitions.setEffect) {
		Blockly.Python.definitions_['import_led_effect'] = 'from clover.srv import SetLEDEffect';
		code += `set_effect_${id} = rospy.ServiceProxy('clover${id}/led/set_effect', SetLEDEffect, persistent=True)\n`;
	}
	if (rosDefinitions.setLeds) {
		Blockly.Python.definitions_['import_set_led'] = 'from led_msgs.srv import SetLEDs\nfrom led_msgs.msg import LEDState';
		code += `set_leds_${id} = rospy.ServiceProxy('clover${id}/led/set_leds', SetLEDs, persistent=True)\n`;
	}
	if (rosDefinitions.ledStateArray) {
		Blockly.Python.definitions_['import_led_state_array'] = 'from led_msgs.msg import LEDStateArray';
	}
	if (rosDefinitions.navigateWait) {
		Blockly.Python.definitions_['import_math'] = 'import math';
		code += NAVIGATE_WAIT(id);
	}
	if (rosDefinitions.landWait) {
		code += LAND_WAIT(id);
	}
	if (rosDefinitions.waitArrival) {
		Blockly.Python.definitions_['import_math'] = 'import math';
		code += WAIT_ARRIVAL(id);
	}
	if (rosDefinitions.arrived) {
		Blockly.Python.definitions_['import_math'] = 'import math';
		code += ARRIVED(id);
	}
	if (rosDefinitions.waitYaw) {
		Blockly.Python.definitions_['import_math'] = 'import math';
		code += WAIT_YAW(id);
	}
	if (rosDefinitions.distance) {
		Blockly.Python.definitions_['import_math'] = 'import math';
		code += GET_DISTANCE(id);
	}
	Blockly.Python.definitions_[`ros_${id}`] = code;
}

function initNode(id) {
	Blockly.Python.definitions_['import_rospy'] = 'import rospy';
	generateROSDefinitions(id);
}

function simpleOffboard(id) {
	rosDefinitions.offboard = true;
	Blockly.Python.definitions_['import_srv'] = IMPORT_SRV;
	initNode(id);
}

// Adjust indentation
Blockly.Python.INDENT = '    ';

export function generateUserCode(workspace) {
	rosDefinitions = {};
	Blockly.Python.STATEMENT_PREFIX = null;
	return Blockly.Python.workspaceToCode(workspace);
}

export function generateCode(workspace) {
	rosDefinitions = {};
	Blockly.Python.STATEMENT_PREFIX = '_b(%1)\n';
	var code = Blockly.Python.workspaceToCode(workspace);
	return code;
}

function buildFrameId(block) {
	let frame = block.getFieldValue('FRAME_ID').toLowerCase();
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	if(frame == 'map'){
		return `'${frame}'`}
	else{
		return `'${frame}${id}'`;
	}
}

Blockly.Python.navigate = function(block) {
	let x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_NONE);
	let y = Blockly.Python.valueToCode(block, 'Y', Blockly.Python.ORDER_NONE);
	let z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_NONE);
	let frameId = buildFrameId(block);
	let speed = Blockly.Python.valueToCode(block, 'SPEED', Blockly.Python.ORDER_NONE);
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);

	let params = [`x=${x}`, `y=${y}`, `z=${z}`, `frame_id=${frameId}`, `speed=${speed}`];

	simpleOffboard(id);

	if (block.getFieldValue('WAIT') == 'TRUE') {
		rosDefinitions.navigateWait = true;
		simpleOffboard(id);

		return `navigate_wait_${id}(${params.join(', ')}),\n`;

	} else {
		if (frameId != 'body${id}') {
			params.push(`yaw=float('nan')`);
		}
		return `navigate_${id}(${params.join(', ')})\n`;
	}
}

Blockly.Python.set_velocity = function(block) {
	let x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_NONE);
	let y = Blockly.Python.valueToCode(block, 'Y', Blockly.Python.ORDER_NONE);
	let z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_NONE);
	let frameId = buildFrameId(block);
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);

	simpleOffboard(id);

	if (frameId == `'body${id}'`) {
		return `set_velocity(vx=${x}, vy=${y}, vz=${z}, frame_id=${frameId})\n`;
	} else {
		return `set_velocity(vx=${x}, vy=${y}, vz=${z}, yaw=float('nan'), frame_id=${frameId})\n`;
	}
}

Blockly.Python.take_off = function(block) {
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	simpleOffboard(id);

	let z = Blockly.Python.valueToCode(block, 'ALT', Blockly.Python.ORDER_NONE);

	if (block.getFieldValue('WAIT') == 'TRUE') {
		rosDefinitions.navigateWait = true;
		simpleOffboard(id);

		return `navigate_wait_${id}(z=${z}, frame_id='body${id}', auto_arm=True)\n`;
	} else {
		return `navigate_${id}(z=${z}, frame_id='body${id}', auto_arm=True)\n`;
	}
}

Blockly.Python.land = function(block) {
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	simpleOffboard(id);

	if (block.getFieldValue('WAIT') == 'TRUE') {
		rosDefinitions.landWait = true;
		simpleOffboard(id);

		return `land_wait_${id}()\n`;
	} else {
		return `land_${id}()\n`;
	}
}

Blockly.Python.angle = function(block) {
	// return [block.getFieldValue('ANGLE'), Blockly.Python.ORDER_UNARY_SIGN];
	Blockly.Python.definitions_['import_math'] = 'import math';
	return [`math.radians(${block.getFieldValue('ANGLE')})`, Blockly.Python.ORDER_FUNCTION_CALL];

}

Blockly.Python.set_yaw = function(block) {
	let yaw = Blockly.Python.valueToCode(block, 'YAW', Blockly.Python.ORDER_NONE);
	let frameId = buildFrameId(block);
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	simpleOffboard(id);
	let code = `navigate_${id}(x=float('nan'), y=float('nan'), z=float('nan'), yaw=${yaw}, frame_id=${frameId})\n`;
	if (block.getFieldValue('WAIT') == 'TRUE') {
		rosDefinitions.waitYaw = true;
		simpleOffboard(id);
		code += `wait_yaw_${id}()\n`;
	}
	return code;
}

Blockly.Python.wait_arrival = function(block) {
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	rosDefinitions.waitArrival = true;
	simpleOffboard(id);
	return `wait_arrival_${id}()\n`;
}

Blockly.Python.get_time = function(block) {
	initNode();
	return ['rospy.get_time()', Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.arrived = function(block) {
	rosDefinitions.arrived = true;
	simpleOffboard();
	return ['arrived()', Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.wait = function(block) {
	initNode(0);
	return `rospy.sleep(${Blockly.Python.valueToCode(block, 'TIME', Blockly.Python.ORDER_NONE)})\n`;
}

Blockly.Python.setpoint = function(block) {
	var type = block.getFieldValue('TYPE');
	let frameId = buildFrameId(block);
	let vx = Blockly.Python.valueToCode(block, 'VX', Blockly.Python.ORDER_NONE);
	let vy = Blockly.Python.valueToCode(block, 'VY', Blockly.Python.ORDER_NONE);
	let vz = Blockly.Python.valueToCode(block, 'VZ', Blockly.Python.ORDER_NONE);
	let yaw = Blockly.Python.valueToCode(block, 'YAW', Blockly.Python.ORDER_NONE);
	let pitch = Blockly.Python.valueToCode(block, 'PITCH', Blockly.Python.ORDER_NONE);
	let roll = Blockly.Python.valueToCode(block, 'ROLL', Blockly.Python.ORDER_NONE);
	let thrust = Blockly.Python.valueToCode(block, 'THRUST', Blockly.Python.ORDER_NONE);
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);

	if (type == 'VELOCITY') {
		rosDefinitions.setVelocity = true;
		simpleOffboard(id);
		return `set_velocity_${id}(vx=${vx}, vy=${vy}, vz=${vz}, frame_id=${frameId}, yaw=float('nan'))\n`;
	} else if (type == 'ATTITUDE') {
		rosDefinitions.setAttitude = true;
		simpleOffboard(id);
		return `set_attitude_${id}(pitch=${pitch}, roll=${roll}, yaw=${yaw}, thrust=${thrust}, frame_id=${frameId})\n`;
	} else if (type == 'RATES') {
		rosDefinitions.setRates = true;
		simpleOffboard(id);
		return `set_rates_${id}(pitch_rate=${pitch}, roll_rate=${roll}, yaw_rate=${yaw}, thrust=${thrust})\n`;
	}
}

Blockly.Python.get_position = function(block) {
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	let frameId = buildFrameId(block);
	simpleOffboard(id);
	var code = `get_telemetry_${id}(${frameId}).${block.getFieldValue('FIELD').toLowerCase()}`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.get_yaw = function(block) {
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	simpleOffboard(id);
	Blockly.Python.definitions_['import_math'] = 'import math';
	let frameId = buildFrameId(block);
	var code = `math.degrees(get_telemetry_${id}(${frameId}).yaw)`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.get_attitude = function(block) {
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	simpleOffboard(id);
	Blockly.Python.definitions_['import_math'] = 'import math';
	var code = `math.degrees(get_telemetry_${id}().${block.getFieldValue('FIELD').toLowerCase()})`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.global_position = function(block) {
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	simpleOffboard(id);
	var code = `get_telemetry_${id}().${block.getFieldValue('FIELD').toLowerCase()}`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.distance = function(block) {
	rosDefinitions.distance = true;
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	simpleOffboard(id);

	let x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_NONE);
	let y = Blockly.Python.valueToCode(block, 'Y', Blockly.Python.ORDER_NONE);
	let z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_NONE);
	let frameId = buildFrameId(block);

	return [`get_distance_${id}(${x}, ${y}, ${z}, ${frameId})`, Blockly.Python.ORDER_FUNCTION_CALL]
}

// What is the rangefinder topic? @mega
Blockly.Python.rangefinder_distance = function(block) {
	initNode();
	Blockly.Python.definitions_['import_range'] = 'from sensor_msgs.msg import Range';
	return [`rospy.wait_for_message('rangefinder/range', Range).range`, Blockly.Python.ORDER_FUNCTION_CALL]
}

Blockly.Python.mode = function(block) {
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	simpleOffboard(id);
	return [`get_telemetry_${id}().mode`, Blockly.Python.ORDER_FUNCTION_CALL]
}

Blockly.Python.armed = function(block) {
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	simpleOffboard(id);
	return [`get_telemetry_${id}().armed`, Blockly.Python.ORDER_FUNCTION_CALL]
}

Blockly.Python.voltage = function(block) {
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	simpleOffboard(id);
	var code = `get_telemetry_${id}().${block.getFieldValue('TYPE').toLowerCase()}`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

function parseColor(color) {
	return {
		r: parseInt(color.substr(2, 2), 16),
		g: parseInt(color.substr(4, 2), 16),
		b: parseInt(color.substr(6, 2), 16)
	}
}

const PARSE_COLOR = `def ${Blockly.Python.FUNCTION_NAME_PLACEHOLDER_}(color):
  return {'r': int(color[1:3], 16), 'g': int(color[3:5], 16), 'b': int(color[5:7], 16)}`;

// TODO: weird code with colour_rgb block
Blockly.Python.set_effect = function(block) {
	rosDefinitions.setEffect = true;
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	initNode(id);

	var effect = block.getFieldValue('EFFECT').toLowerCase();

	if (effect == 'rainbow' || effect == 'rainbow_fill') {
		return `set_effect_${id}(effect='${effect}')\n`;
	} else {
		let colorCode = Blockly.Python.valueToCode(block, 'COLOR', Blockly.Python.ORDER_NONE);

		if (/^'(.*)'$/.test(colorCode)) { // is simple string
			let color = parseColor(colorCode);
			return `set_effect_${id}(effect='${effect}', r=${color.r}, g=${color.g}, b=${color.b})\n`;
		} else {
			let parseColor = Blockly.Python.provideFunction_('parse_color', [PARSE_COLOR]);
			return `set_effect_${id}(effect='${effect}', **${parseColor}(${colorCode}))\n`;
		}
	}
}

Blockly.Python.set_led = function(block) {
	rosDefinitions.setLeds = true;
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	initNode();

	var index = Blockly.Python.valueToCode(block, 'INDEX', Blockly.Python.ORDER_NONE);
	var colorCode = Blockly.Python.valueToCode(block, 'COLOR', Blockly.Python.ORDER_NONE);

	if (/^'(.*)'$/.test(colorCode)) { // is simple string
		let color = parseColor(colorCode);
		return `set_leds_${id}([LEDState(index=int(${index}), r=${color.r}, g=${color.g}, b=${color.b})])\n`; // TODO: check for simple int
	} else {
		let parseColor = Blockly.Python.provideFunction_('parse_color', [PARSE_COLOR]);
		return `set_leds_${id}([LEDState(index=${index}, **${parseColor}(${colorCode}))])\n`;
	}
}

function GET_LED_COUNT(id) { 
	return `led_count = None\ndef get_led_count_${id}():
    global led_count
    if led_count is None:
        led_count = len(rospy.wait_for_message('clover${id}/led/state', LEDStateArray, timeout=10).leds)
    return led_count\n`;
	}

Blockly.Python.led_count = function(block) {
	rosDefinitions.ledStateArray = true;
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	initNode(id);
	Blockly.Python.definitions_['get_led_count'] = GET_LED_COUNT(id);
	return [`get_led_count_${id}()`, Blockly.Python.ORDER_FUNCTION_CALL]
}

//! FORMS (FORMA????O)
// defini????o
Blockly.Python.forms = function(block) {
	// ativa os n??s principais do simplesOffboard (do pr??prio ROS j??)
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	simpleOffboard(id);
	// ativa o navigateWait, bem como ativamos o simpleOffboard acima
	rosDefinitions.navigateWait = true;
	// define os parametros do bloco um a um
	// liga o valor de z e x com os blocos espectivos Z (altura) e X (lado), esses valores er??o setados no bloco
	var value_z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_NONE);
	var value_x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_NONE);
	// pega a string dos formatos (SQAURE, TRIANGLE ...)
	let dropdown_forms = block.getFieldValue('Forms');
	// Id do bloco (n??o fica vis??vel nos blocos)
	let frameId = buildFrameId(block);
	// outro parametro que sera setado no bloco
	let speed = Blockly.Python.valueToCode(block, 'SPEED', Blockly.Python.ORDER_NONE);	// define os parametros que ser??o retornados de acordo com as variaveis setadas aciam	// x e z sao fornecidos pelo bloco, y ser?? nulo para essa transla????o 	let params = [`x=${value_x}`, `y=0`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];	return `navigate_wait(${params.join(', ')}),\n`;	
	// op????o para os formatos, dependendo de cada um ele setar?? os parametros uma certa forma
	// quadrado de 4 lados, com lado X
	if (dropdown_forms == 'SQUARE'){
	let params = [`x=${value_x}`, `y=0`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];
	let params2 = [`x=${value_x}`, `y=${value_x}`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];
	let params3 = [`x=0`, `y=${value_x}`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];
	let params4 = [`x=0`, `y=0`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];
	return `navigate_wait(${params.join(', ')}),\nnavigate_wait(${params2.join(', ')}),\nnavigate_wait(${params3.join(', ')}),\nnavigate_wait(${params4.join(', ')}),\n`;	
	}
	// triangulo is??celes com base X 
	if (dropdown_forms == 'TRIANGLE'){
	let params = [`x=${value_x}`, `y=0`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`]
	let params2 = [`x=(${value_x}/2)`, `y=(${value_x}/2)*math.sqrt(3)`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];
	let params3 = [`x=0`, `y=0`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];
	return `navigate_wait(${params.join(', ')}),\nnavigate_wait(${params2.join(', ')}),\nnavigate_wait(${params3.join(', ')}),\n`;
	}
	// hexagono is??celes regular de base X
	if (dropdown_forms == 'HEXAG'){
	let params = [`x=${value_x}`, `y=0`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`]
	let params2 = [`x=(5*${value_x}/4)`, `y=(${value_x}/2)*math.sqrt(3)`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];
	let params3 = [`x=${value_x}`, `y=(${value_x})*math.sqrt(3)`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`]
	let params4 = [`x=0`, `y=(${value_x})*math.sqrt(3)`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];
	let params5 = [`x=(-${value_x}/4)`, `y=(${value_x}/2)*math.sqrt(3)`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];
	let params6 = [`x=0`, `y=0`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];
	return `navigate_wait(${params.join(', ')}),\nnavigate_wait(${params2.join(', ')}),\nnavigate_wait(${params3.join(', ')}),\nnavigate_wait(${params4.join(', ')}),\nnavigate_wait(${params5.join(', ')}),\nnavigate_wait(${params6.join(', ')}),\n`;
	}
	// obs: no return foi usado uma "gambiarra", v??rios navigate_wait s??o concatenados com ",\n" entre eles,
	// dessa forma ?? poss??vel montar uma lista de intru????es para serem seguidas
}

// Swarm blocks

function initSwarm(){
	Blockly.Python.definitions_['import_swarm'] = 'from swarm_in_blocks import swarm as sw'
	Blockly.Python.definitions_['init_swarm'] = 'swarm = sw.Swarm()\n'
	Blockly.Python.definitions_['init_mode']= 'swarm.startSimulation()\n'
}

Blockly.Python['take_off_all'] = function(block) {
	initSwarm()
	var z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_ATOMIC);
	// TODO: Assemble Python into code variable.
	var code = `swarm.takeOffAll(z=${z})\n`;
	return code;
};

Blockly.Python['land_all'] = function(block) {
	initSwarm()
	// TODO: Assemble Python into code variable.
	var code = `swarm.landAll()\n`;
	return code;
};

Blockly.Python['return_and_land'] = function(block) {
	initSwarm()
	// TODO: Assemble Python into code variable.
	var code = 'swarm.returnAndLand()\n';
	return code;
};

Blockly.Python['set_2d_formation'] = function(block) {
	initSwarm()
	var formation = block.getFieldValue('2D formation');
	var num = Blockly.Python.valueToCode(block, 'NUM', Blockly.Python.ORDER_ATOMIC);
	var length = Blockly.Python.valueToCode(block, 'LENGTH', Blockly.Python.ORDER_ATOMIC);
	// TODO: Assemble Python into code variable.
	var code = `swarm.setFormation2D('${formation.toLowerCase()}',${num},${length})\n`;
	return code;
};

Blockly.Python['set_3d_formation'] = function(block) {
	initSwarm()
	var formation = block.getFieldValue('3D formation');
	var num = Blockly.Python.valueToCode(block, 'NUM', Blockly.Python.ORDER_ATOMIC);
	var length = Blockly.Python.valueToCode(block, 'LENGTH', Blockly.Python.ORDER_ATOMIC);
	// TODO: Assemble Python into code variable.
	var code = `swarm.setFormation3D('${formation.toLowerCase()}',${num},${length})\n`;
	return code;
};

Blockly.Python['translate_formation'] = function(block) {
	initSwarm()
	var tx = Blockly.Python.valueToCode(block, 'TX', Blockly.Python.ORDER_ATOMIC);
	var ty = Blockly.Python.valueToCode(block, 'TY', Blockly.Python.ORDER_ATOMIC);
	var tz = Blockly.Python.valueToCode(block, 'TZ', Blockly.Python.ORDER_ATOMIC);
	// TODO: Assemble Python into code variable.
	var code = `swarm.translateFormation(${tx}, ${ty}, ${tz})\n`;
	return code;
};

Blockly.Python['rotate_formation'] = function(block) {
	initSwarm()
	var anglex = Blockly.Python.valueToCode(block, 'ANGLEX', Blockly.Python.ORDER_ATOMIC);
	var angley = Blockly.Python.valueToCode(block, 'ANGLEY', Blockly.Python.ORDER_ATOMIC);
	var anglez = Blockly.Python.valueToCode(block, 'ANGLEZ', Blockly.Python.ORDER_ATOMIC);
	// TODO: Assemble Python into code variable.
	var code = `swarm.rotateFormation(${anglex}, ${angley}, ${anglez})\n`;
	return code;
};

Blockly.Python['scale_formation'] = function(block) {
	initSwarm()
	var sx = Blockly.Python.valueToCode(block, 'SX', Blockly.Python.ORDER_ATOMIC);
	var sy = Blockly.Python.valueToCode(block, 'SY', Blockly.Python.ORDER_ATOMIC);
	var sz = Blockly.Python.valueToCode(block, 'SZ', Blockly.Python.ORDER_ATOMIC);
	// TODO: Assemble Python into code variable.
	var code = `swarm.scaleFormation(${sx}, ${sy}, ${sz})\n`;
	return code;
};

Blockly.Python['apply_formation'] = function(block) {
	initSwarm()
	// TODO: Assemble Python into code variable.
	var code = 'swarm.applyFormation()\n';
	return code;
};

Blockly.Python['plot_formation'] = function(block) {
	initSwarm()
	// TODO: Assemble Python into code variable.
	var type = block.getFieldValue('PLOT_TYPE');
	var code = `swarm.plotPreview(plot_type='${type}')\n`;
	return code;
};