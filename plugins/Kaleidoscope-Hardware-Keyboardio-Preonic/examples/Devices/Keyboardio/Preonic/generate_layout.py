#!/usr/bin/env python3

import re
import os
import argparse

class LayoutConfig:
    def __init__(self, mode='stickers'):
        if mode == 'stickers':
            # Sticker mode - white on black with bright colors
            self.mode = 'stickers'
            self.key_width = 32
            self.key_height = 32
            self.key_gap = 10
            self.padding = 30
            self.text_size = 6.25
            self.legend_offset = 3
            self.icon_to_text_offset = 2
            self.icon_to_icon_offset = 4
            self.scale_factor = 1.33
            self.background_color = "#000000"
            self.key_background = "#111111"
            self.text_color = "#ffffff"
            self.stroke_color = "#333333"
            self.show_butterfly = True
            self.layer_colors = {
                'main': '#ffffff',   # QWERTY layer - white
                'lower': '#66b3ff',  # LOWER layer - blue
                'raise': '#ff944d',  # RAISE layer - orange
                'fun': '#66ff66'     # FUN layer - green
            }
        else:  # layout-card mode
            # Layout card mode - black on white with darker colors
            self.mode = 'layout-card'
            self.key_width = 32
            self.key_height = 32
            self.key_gap = 15  # Wider gaps for better readability
            self.padding = 40
            self.text_size = 6.75  # Slightly larger text
            self.legend_offset = 1
            self.icon_to_text_offset = 2.5
            self.icon_to_icon_offset = 5
            self.scale_factor = 1.5  # Larger overall size
            self.background_color = "#ffffff"
            self.key_background = "#f8f8f8"
            self.text_color = "#000000"
            self.stroke_color = "#dddddd"  # 50% lighter
            self.show_butterfly = False
            self.layer_colors = {
                'main': '#000000',   # QWERTY layer - black
                'lower': '#0066cc',  # LOWER layer - darker blue
                'raise': '#e27857',  # RAISE layer - darker orange
                'fun': '#006600'     # FUN layer - darker green
            }
        
        # Calculate derived dimensions
        self.keyboard_width = 12 * (self.key_width + self.key_gap) - self.key_gap
        self.keyboard_height = 6 * (self.key_height + self.key_gap) - self.key_gap

def parse_keymap(ino_file):
    with open(ino_file, 'r') as f:
        content = f.read()
    
    # Find the KEYMAPS section
    keymap_match = re.search(r'KEYMAPS\((.*?)\);', content, re.DOTALL)
    if not keymap_match:
        return None
    
    layers = {}
    current_layer = None
    
    # Process each line in the keymap
    for line in keymap_match.group(1).split('\n'):
        # Check for layer definition
        layer_match = re.match(r'\s*\[(\w+)\]\s*=\s*KEYMAP', line)
        if layer_match:
            current_layer = layer_match.group(1)
            layers[current_layer] = []
            continue
        
        # Skip lines without keys
        if not current_layer or not line.strip() or '//' in line:
            continue
        
        # Extract keys from the line, handling parentheses properly
        keys = []
        current_key = ''
        paren_count = 0
        
        for char in line.strip():
            if char == '(' and not current_key:
                continue
            elif char == '(':
                paren_count += 1
                current_key += char
            elif char == ')':
                paren_count -= 1
                current_key += char
                if paren_count == 0 and current_key:
                    keys.append(current_key.strip())
                    current_key = ''
            elif char == ',' and paren_count == 0:
                if current_key:
                    keys.append(current_key.strip())
                current_key = ''
            else:
                current_key += char
        
        if current_key:
            keys.append(current_key.strip())
        
        # Filter out empty keys and structural elements
        keys = [k for k in keys if k and k != '(' and k != ')' and k != 'KEYMAP']
        if keys:
            layers[current_layer].extend(keys)
    
    return layers

def get_mouse_icon(button=None):
    path = '''<path d="m6.0006 9.9714v10c0 4 12 4 12 0v-10s0-4-6-4c-6 0-6 4-6 4z"/>
<path d="m11.656 6.8328s0-3 1-4 2-2-1-2" stroke-width="1.5"/>'''

    if button is None:
        return path
    else:
        # Add button indicator
        button_x = {'left': 8, 'middle': 11.25, 'right': 14.5}[button]
        return f'''<g transform="translate(-4,0)">
{path}
<rect x="{button_x}" y="8" width="1.5" height="5" fill="currentColor" opacity="1"/>
</g>'''

def get_bluetooth_icon():
    return ' <path d="M7.32,8.1l9.35,7.8-4.68,3.9V4.2l4.68,3.9-9.35,7.8"/>'
    
def get_bluetooth_off_icon():
    return get_bluetooth_icon() + '<path d="M4,4 L20,20" stroke="currentColor" stroke-width="2"/>'

def get_bluetooth_pair_icon():
    return '''<g transform="translate(0,0)">
    <path d="M7.32,8.1l9.35,7.8-4.68,3.9V4.2l4.68,3.9-9.35,7.8"/>
    <path d="M28,9 a5,5 0 1,0 10,0 a5,5 0 1,0 -10,0 M28,12 L22,17" />
</g>'''

def get_volume_up_icon():
    return '''<path d="M12,7 L12,17 L7,13 L4,13 L4,11 L7,11 L12,7 Z" rx="1"/>
<path d="M15,9 C17,11 17,13 15,15" rx="0.5"/>
<path d="M17,7 C20,10 20,14 17,17" rx="0.5"/>'''

def get_volume_down_icon():
    return '''<path d="M12,7 L12,17 L7,13 L4,13 L4,11 L7,11 L12,7 Z" rx="1"/>
<path d="M15,9 C17,11 17,13 15,15" rx="0.5"/>'''

def get_mute_icon():
    return '''<path d="M12,7 L12,17 L7,13 L4,13 L4,11 L7,11 L12,7 Z" rx="1"/>
<line x1="15" y1="9" x2="20" y2="15"/>
<line x1="20" y1="9" x2="15" y2="15"/>'''

def get_prev_track_icon():
    return '''<path d="m6.2286 6v12" stroke-linecap="round"/>
<path rx="1" d="m18.8 6-9 6 9 6z" stroke-linejoin="round"/>'''

def get_next_track_icon():
    return '''<path d="m18.8 6v12" stroke-linecap="round"/>
<path rx="1" d="m6.2286 6 9 6-9 6z" stroke-linejoin="round"/>'''

def get_led_effect_icon():
    return '''<circle cx="12" cy="12" r="5"/>
<path d="M12,3 L12,7 M21,12 L17,12 M12,21 L12,17 M3,12 L7,12"/>
<path d="M18.4,5.6 L15.6,8.4 M18.4,18.4 L15.6,15.6 M5.6,18.4 L8.4,15.6 M5.6,5.6 L8.4,8.4" stroke-width="1.5"/>'''

def get_play_pause_icon():
    return '''<g stroke-linecap="round" stroke-linejoin="round">
<path rx="1" d="m7 6v12l7-6z"/>
<path d="m16.2 6v12m3-12v12"/>
</g>'''

def get_volume_arrow(direction):
    return f'''<path d="M12,{20 if direction == 'down' else 4} L8,{14 if direction == 'down' else 10} L16,{14 if direction == 'down' else 10} Z"/>'''

def get_keyclick_icon():
    return '''<rect x="6" y="8" width="8" height="8" rx="1"/>
<path d="M16,10 C18,12 18,12 16,14" stroke-width="1.5"/>
<path d="M18,8 C21,12 21,12 18,16" stroke-width="1.5"/>'''

def get_warp_icon(quadrant):
    # quadrant can be 'nw', 'ne', 'sw', 'se'
    rect_size = 8
    gap = 2
    total_size = 2 * rect_size + gap
    
    # Map quadrant to which rectangle should be filled
    fill_coords = {
        'nw': (0, 0),           # Top-left quadrant
        'ne': (rect_size + gap, 0),    # Top-right quadrant
        'sw': (0, rect_size + gap),    # Bottom-left quadrant
        'se': (rect_size + gap, rect_size + gap)   # Bottom-right quadrant
    }[quadrant]
    
    mouse_width = get_svg_path_width(get_mouse_icon())
    
    return f'''<g transform="translate({-mouse_width},0)">
{get_mouse_icon()}
</g>
<g transform="translate(6,0)">
    <rect x="4" y="4" width="{total_size}" height="{total_size}" fill="none" stroke="currentColor"/>
    <rect x="{fill_coords[0]+4 }" y="{fill_coords[1]+4}" width="{rect_size}" height="{rect_size}" fill="currentColor"/>
</g>'''

def get_warp_end_icon():
    rect_size = 8
    gap = 2
    total_size = 2 * rect_size + gap
    mouse_width = get_svg_path_width(get_mouse_icon())
    return f'''<g transform="translate({-mouse_width},0)">
{get_mouse_icon()}
</g>
<g transform="translate(6,0)">
    <rect x="4" y="4" width="{total_size}" height="{total_size}" fill="none" stroke="currentColor"/>
</g>'''

def get_usb_icon():
    return '''<path d="M8,4 L16,4 L16,8 L18,8 L18,16 L14,16 L14,20 L10,20 L10,16 L6,16 L6,8 L8,8 Z" rx="1"/>
<path d="M10,4 L10,8 M14,4 L14,8" stroke-width="1"/>'''

def get_magnifying_glass_icon():
    return '<path d="M9,9 a4,4 0 1,0 8,0 a4,4 0 1,0 -8,0 M15,15 L19,19" stroke="currentColor" stroke-width="2" fill="none" transform="translate(22,0) scale(-1,1)"/>'

def clean_key_name(key):
    if not key or key == '___' or key == 'XXX':
        return ''
    
    # Remove any extra spaces
    key = key.strip()
    
    # Special handling for layer keys - return tuple of (text, css_class)
    mo_match = re.match(r'ShiftToLayer\((\w+)\)', key.replace(' ', ''))
    if mo_match:
        layer_name = mo_match.group(1).title()

        return (layer_name, f'layer-{layer_name.lower()}')
    
    # Handle LSHIFT() wrapped keys
    lshift_match = re.match(r'LSHIFT\(Key_(.+?)\)', key)
    if lshift_match:
        shifted_chars = {
            '1': '!', '2': '@', '3': '#', '4': '$', '5': '%',
            '6': '^', '7': '&', '8': '*', '9': '(', '0': ')',
            'Minus': '_', 'Equals': '+',
            'LeftBracket': '{', 'RightBracket': '}',
            'Backslash': '|', 'Semicolon': ':', 'Quote': '"',
            'Comma': '<', 'Period': '>', 'Slash': '?',
            'Backtick': '~'
        }
        base_key = lshift_match.group(1)
        return shifted_chars.get(base_key, f'⇧{base_key}')

    # Common key name mappings
    mappings = {
        'Key_LeftArrow': '←',
        'Key_RightArrow': '→',
        'Key_UpArrow': '↑',
        'Key_DownArrow': '↓',
        'Key_Backspace': 'Bksp',
        'Key_Enter': 'Enter',
        'Key_Escape': 'Esc',
        'Key_Delete': 'Del',
        'Key_LeftControl': 'Ctrl',
        'Key_LeftAlt': 'Alt',
        'Key_RightAlt': 'AltGr',
        'Key_LeftGui': 'Cmd',
        'Key_Space': 'Space',
        'Key_Tab': 'Tab',
        'Key_Star': '*',
        'Key_Plus': '+',
        'Key_Backtick': '`',
        'Key_Semicolon': ';',
        'Key_Quote': "'",
        'Key_Comma': ',',
        'Key_Period': '.',
        'Key_Slash': '/',
        'Key_Equals': '=',
        'Key_Minus': '-',
        'Key_LeftBracket': '[',
        'Key_RightBracket': ']',
        'Key_Backslash': '\\',
        'Key_CapsLock': 'Caps',
        'Key_LeftShift': 'Shift',
        'Consumer_ScanNextTrack': get_next_track_icon(),
        'Consumer_ScanPreviousTrack': get_prev_track_icon(),
        'Consumer_Mute': get_mute_icon(),
        'Consumer_VolumeIncrement': get_volume_up_icon(),
        'Consumer_VolumeDecrement': get_volume_down_icon(),
        'Key_mouseBtnL': get_mouse_icon('left'),
        'Key_mouseBtnM': get_mouse_icon('middle'),
        'Key_mouseBtnR': get_mouse_icon('right'),
        'Key_mouseWarpNW': get_warp_icon('nw'),
        'Key_mouseWarpNE': get_warp_icon('ne'),
        'Key_mouseWarpSW': get_warp_icon('sw'),
        'Key_mouseWarpSE': get_warp_icon('se'),
        'Key_mouseWarpEnd': get_warp_end_icon(),
        'Key_mouseUp': get_mouse_icon() + ' + ↑',
        'Key_mouseDown': get_mouse_icon() + ' + ↓',
        'Key_mouseLeft': get_mouse_icon() + ' + ←',
        'Key_mouseRight': get_mouse_icon() + ' + →',
        'Key_mouseL': get_mouse_icon() + ' + ←',
        'Key_mouseDn': get_mouse_icon() + ' + ↓',
        'Key_mouseR': get_mouse_icon() + ' + →',
        'Key_LEDEffectNext': get_led_effect_icon(),
        'M(MACRO_BT_OFF)': get_usb_icon(),
        'M(MACRO_BT_SELECT_1)': get_bluetooth_icon() + ' + 1',
        'M(MACRO_BT_SELECT_2)': get_bluetooth_icon() + ' + 2',
        'M(MACRO_BT_SELECT_3)': get_bluetooth_icon() + ' + 3',
        'M(MACRO_BT_SELECT_4)': get_bluetooth_icon() + ' + 4',
        'M(MACRO_BT_PAIR)': get_bluetooth_pair_icon(),
        'M(MACRO_FLASH_ERASE)': '⚡',
        'M(MACRO_ANY)': 'Any',
        'Key_Hyper': 'Hyper',
        'Key_ToggleKeyclick': get_keyclick_icon(),
    }
    
    # Check mappings first
    if key in mappings:
        return mappings[key]
    
    # Handle single letters and numbers
    if key.startswith('Key_') and len(key) == 5:
        return key[4]
    
    # Remove common prefixes
    prefixes = ['Key_', 'Consumer_', 'M(MACRO_', ')']
    for prefix in prefixes:
        if key.startswith(prefix):
            key = key[len(prefix):]
    
    return key

def generate_svg(layers, config):
    # Centered paths for different key types
    if (config.mode == 'stickers'):
        dsa_key = '''M31.98,28.85c-.16,1.71-1.36,2.98-3.09,3.12-3.97.37-8.43.59-12.42.6h-.02c-4.13-.02-8.31-.2-12.42-.6h.03c-1.7-.16-2.99-1.36-3.11-3.09h-.01c-.37-3.97-.6-8.43-.59-12.42-.01-4.14.18-8.32.6-12.43v.03c.17-1.72,1.33-2.95,3.08-3.11h0c4.13-.4,8.28-.6,12.43-.6h0c4.13,0,8.31.19,12.42.6h-.03c1.68.12,3.02,1.39,3.1,3.09h.02c.4,4.12.6,8.26.6,12.41h-.01c0,4.14-.18,8.32-.6,12.43'''
        homing_key = '''M28.1,30.72c1.35-.18,2.39-1.24,2.57-2.58h0c1.4-7.96,1.43-16.19,0-24.16h0c-.17-1.32-1.21-2.38-2.55-2.55h.01c-3.98-.73-8.04-1.07-12.09-1.08h.02c-4.06-.02-8.13.34-12.12,1.08h.03c-1.33.18-2.4,1.22-2.55,2.57h0C-.01,11.96,0,20.19,1.41,28.16h0c.18,1.32,1.21,2.38,2.55,2.55h-.01c7.97,1.44,16.23,1.47,24.19,0'''
        space_key = '''M85.31.35H1.12h0c-.68-.04-.8.45-.78,1.04.71,10.07.76,20.54,0,30.61,0,.35-.02.71.15,1.01.15.18.39.24.63.22h0c.62-.21,84.4.4,84.81-.21.12-.23.17-.5.16-.78-.3-4.31-.51-9.43-.55-13.76-.04-5.49.17-11.62.55-17.1,0-.29-.03-.59-.17-.82-.16-.17-.38-.22-.63-.21'''

    else:
        dsa_key = 'M-2 -2 H36 V36 H-2 Z'
        homing_key = "M-2 -2 Q17 2 36 -2 Q34 17 36 36 Q17 34 -2 36 Q2 17 -2 -2 Z"
        space_key = 'M-2 -2 H84.25 V36 H-2 Z'
    # Centered 2U spacebar path

    # Centered homing key path

    # Centered knob path
    knob_path = '''M39.45,19.9C39.45,9.1,30.7.35,19.9.35S.35,9.1.35,19.9s8.75,19.55,19.55,19.55,19.55-8.75,19.55-19.55'''



    def render_key(idx, row, col, is_space_key=False, is_homing=False):

        x = col * (config.key_width + config.key_gap)
        y = row * (config.key_height + config.key_gap)
        
        # Check if this is the volume knob
        if row == 0 and col == 11:
            return f'''        <g transform="translate({x + config.key_width/2}, { y+config.key_gap })">
                <path d="{knob_path}" fill="none" stroke="{config.stroke_color}" stroke-width=".7"/>
                <path id="curve_{row}_{col}" fill="none" d="M-10,0 A10,10 0 0,1 10,0" />
               
                <g transform="translate({config.key_width/2+4}, {config.key_height/2+2})">
                 <text fill="{config.text_color}" font-size="{config.text_size}">
                    <textPath href="#curve_{row}_{col}" startOffset="50%" text-anchor="middle">-VOL+</textPath>
                </text>
                </g>
                <g transform="translate({13}, {15}) scale(0.5)" fill="none" stroke="{config.text_color}" stroke-width="1">
                    {get_play_pause_icon()}
                </g>
            </g>
'''
        
        # Skip rendering the first two keys in row 0 (they're now part of the knob display)
        if row == 0 and (col == 7 or col == 8):
            return ''
            
        # Handle space key (double width)
        if is_space_key:
            space_width = get_svg_path_width(space_key)
            if config.mode == 'stickers':
                end_offset = space_width-19
            else:
                end_offset = space_width-config.legend_offset*4
            return f'''        <g transform="translate({x + config.key_gap }, {y + config.key_height/2})">
                <path d="{space_key}" fill="none" stroke="{config.stroke_color}" stroke-width=".7"/>
                <text x="{config.legend_offset}" y="{config.key_height-config.legend_offset}" fill="{config.text_color}" font-size="{config.text_size}" text-anchor="bottom" >Space</text>
                <text x="{end_offset}" y="{config.key_height-config.legend_offset}" fill="{config.layer_colors['lower']}" font-size="{config.text_size}" text-anchor="end">Bksp</text>
            </g>
'''

        # Get key values from layers
        qwerty_key = layers['QWERTY'][idx] if idx < len(layers['QWERTY']) else ''
        lower_key = layers['LOWER'][idx] if idx < len(layers.get('LOWER', [])) else ''
        raise_key = layers['RAISE'][idx] if idx < len(layers.get('RAISE', [])) else ''
        fun_key = layers['FUN'][idx] if idx < len(layers.get('FUN', [])) else ''
        
        # Clean up key names
        qwerty_key = clean_key_name(qwerty_key)
        lower_key = clean_key_name(lower_key)
        raise_key = clean_key_name(raise_key)
        fun_key = clean_key_name(fun_key)

        # Use homing key path for F and J keys
        key_path = homing_key if is_homing else dsa_key
        stroke_color = config.stroke_color 

        svg = f'''        <g transform="translate({x + config.key_width/2}, {y + config.key_height/2})">
            <path d="{key_path}" fill="none" stroke="{stroke_color}" stroke-width=".7"/>
'''

        def key_label(content, x, y, layer='main', color_override=None, scale=0.3):  
            color = config.layer_colors[layer]
            if color_override:
                color = color_override
            text_anchor = "start" if layer in ['lower', 'raise'] else "start"

            # Check if this is a mouse warp icon
            if isinstance(content, str) and content.startswith('<path'):
                if 'fill="currentColor"' in content:  # Mouse warp icon with filled quadrant
                    content = content.replace('fill="currentColor"', f'fill="{config.stroke_color}"')
            
            # Check if content is an icon (SVG path/group) or text
            is_icon = any(content.startswith(prefix) for prefix in ['<path', '<g', '<rect', '<circle'])
            
            if ' + ' in content:  # Compound icon with text
                icon, text = content.split(' + ', 1)
                icon_width = get_svg_path_width(icon) * scale
                icon_height = get_svg_path_height(icon) * scale
                x_pos = x if layer in ['qwerty', 'fun'] else (x-icon_width-icon_width)
                text_x_pos = x_pos +icon_width+config.icon_to_text_offset if layer in ['qwerty', 'fun'] else x_pos+icon_width+config.icon_to_text_offset

                return f'''            <g transform="translate({x_pos},{y-icon_height}) scale({scale})" fill="none" stroke="{color}" stroke-width="2">
                {icon}
            </g>
            <text x="{text_x_pos}" y="{y}" fill="{color}" font-size="{config.text_size}" text-anchor="{text_anchor}">{text}</text>
'''

            elif is_icon:  # Single icon
                icon_width = get_svg_path_width(content) * scale
                icon_height = get_svg_path_height(content) * scale
                x_pos = config.legend_offset if layer in ['qwerty', 'fun'] else config.key_width-icon_width

                return f'''            <g transform="translate({x_pos},{config.legend_offset+3.25}) scale({scale})" fill="none" stroke="{color}" color="{color}" text-anchor="{text_anchor}" stroke-width="2">
                {content}
            </g>
'''
            else:  # Plain text
                text_anchor = "end" if layer in ['lower', 'raise'] else "start"
                return f'''            <text x="{x}" y="{y}" fill="{color}" font-size="{config.text_size}" text-anchor="{text_anchor}">{content}</text>
'''
        
        # Add text elements for each layer with smaller font sizes and adjusted positions
        if qwerty_key:
            if isinstance(qwerty_key, tuple):
                text, css_class = qwerty_key
                # strip layer- from the css class
                css_class = css_class.replace('layer-', '')
                color = config.layer_colors[css_class]
                svg += key_label(text, config.legend_offset, config.key_height-config.legend_offset, 'main', color)
            elif qwerty_key != '___':
                svg += key_label(qwerty_key, config.legend_offset, config.key_height-config.legend_offset, 'main')

        if lower_key and lower_key != '___':
            svg += key_label(lower_key, config.key_width-config.legend_offset, config.key_height-config.legend_offset, 'lower')

        if raise_key and raise_key != '___':
            svg += key_label(raise_key, config.key_width-config.legend_offset, config.text_size+3, 'raise')

        if fun_key and fun_key != '___':
            svg += key_label(fun_key, config.legend_offset, config.text_size+3, 'fun')
        
        svg += '''        </g>
'''
        return svg


    scale_factor = config.scale_factor
    svg_template = f'''<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<svg width="{scale_factor * (   config.keyboard_width + 3*config.padding)}" height="{scale_factor * (config.keyboard_height + 3*config.padding)}" 
     xmlns="http://www.w3.org/2000/svg">
    <style>
        text {{ font-family: 'Gorton Perfected', sans-serif;
        font-weight: 600;}}
    </style>
    <g transform="scale({scale_factor})">
    <rect x="{0}" y="{0}" width="{config.keyboard_width + 3*config.padding}" height="{config.keyboard_height + 3*config.padding}" fill="{config.background_color}"/>
    <rect x="{config.padding  }" y="{config.padding  }" width="{config.keyboard_width+  config.padding}" height="{config.keyboard_height + config.padding}" rx="8" fill="{config.background_color}"/>
    <g transform="translate({config.padding}, {config.padding})">
    <!-- Logo -->
              {get_logo_element(config)}
    
'''

    # Add the first row (5 keys)
    y_offset = 0
    for i in range(5):
        svg_template += render_key(i, 0, i + 7)
    
    # Add the main 5x12 grid
    start_idx = 5  # Start after the first row (5 keys)
    for row in range(5):  # 5 rows
        for col in range(12):  # 12 columns
            idx = start_idx + row * 12 + col
            is_space_key = (row == 4 and (col == 5 or col == 6))  # Space key is in row 4 (5th row)
            is_homing = (row == 2 and (col == 4 or col == 7))  # F and J keys
            if is_space_key and col == 5:  # Only add the key once
                svg_template += render_key(idx, row + 1, col, is_space_key=True)
                continue
            elif is_space_key and col == 6:
                continue  # Skip the second key of the space bar
            svg_template += render_key(idx, row + 1, col, is_homing=is_homing)
    
    svg_template += '</g></g></svg>'
    return svg_template

def get_logo_element(config):
    """Get the logo element - either butterfly or logo.png"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    logo_path = os.path.join(script_dir, 'logo.png')
    
    if not config.show_butterfly and os.path.exists(logo_path):
        # Convert 20mm to pixels (assuming 96 DPI)
        height_px = 25
        return f'''    <g transform="translate({config.key_width+8}, {config.key_gap*1.5}) ">
<image href="logo.png" height="{height_px}" preserveAspectRatio="xMidYMid meet"/>
</g>
<text x="{config.key_width+150}" y="{config.key_gap*2.75}" font-size="18" font-weight="bold" fill="{config.text_color}">Preonic</text>'''
    elif config.show_butterfly:
        
        return f'''
            <g transform="translate({config.key_width+(config.key_gap)}, {config.key_gap*2}) ">
<circle cx="11.875" cy="9.5" r="19.55" fill="none" stroke="{config.stroke_color}" stroke-width=".7" />
        <g transform="scale(0.028)">
            <path fill="#ffffff" stroke="#333333" stroke-width="3" d="M422.6 261.1c-21.2 0-38.7 35-38.7 95.2 0 60.3 15.6 188.3 38.7 188.3 23.1 0 38.7-128 38.7-188.3 0-60.3-17.5-95.2-38.7-95.2"/>
            <path fill="#ffffff" stroke="#333333" stroke-width="3" transform="matrix(.8869 -.4619 .4619 .8869 -55.651 199.1881)" d="M369.5 176.6h19v73.4h-19z"/>
            <path fill="#ffffff" stroke="#333333" stroke-width="3" transform="matrix(.4618 -.887 .887 .4618 61.4743 527.8238)" d="M429 203.8h73.4v19H429z"/>
            <path fill="#ffffff" stroke="#333333" stroke-width="3" d="M58 90.4C17.2 80.7-6.2 105.3 1.2 154c15.5 101.9 52 164.5 129.9 183.5 59.3 14.4 221.8 20 221.8 20-33.3-162.4-195.1-243.4-294.9-267.1"/>
            <path fill="#ffffff" stroke="#333333" stroke-width="3" d="M355 389.5s-71.1-5.6-119-1.6c-61 5.1-98 25.6-98 96.2 0 103.5 99.5 150.2 150.4 147.5 20.9-1.1 56.9-44.1 69.5-77.5 8.3-21.9 11.9-42.3 8.9-73.9-1.8-19.4-11.8-90.7-11.8-90.7"/>
            <path fill="#ffffff" stroke="#333333" stroke-width="3" d="M786.7 90.4c-99.8 23.7-261.7 104.7-295 267.1 0 0 162.6-5.6 221.8-20 77.9-19 114.4-81.6 129.9-183.5 7.5-48.7-15.9-73.3-56.7-63.6"/>
            <path fill="#ffffff" stroke="#333333" stroke-width="3" d="M608.7 387.9c-47.8-4-119 1.6-119 1.6s-10 71.4-11.8 90.7c-3 31.6.7 51.9 8.9 73.9 12.6 33.4 48.5 76.4 69.5 77.5 50.8 2.7 150.4-44 150.4-147.5 0-70.6-37-91.1-98-96.2"/>
        </g>
        </g>'''
    else:
        return ''  # No logo if butterfly is disabled and logo.png doesn't exist

def get_svg_path_width(path_d):
    """
    Calculate the width of an SVG path by extracting and analyzing its coordinates.

    Args:
        path_d (str): The `d` attribute of the SVG path.

    Returns:
        float: The width of the path (difference between max and min x-coordinates).
    """
    # Extract all coordinates using regex
    coords = re.findall(r"[-+]?\d*\.?\d+", path_d)
    coords = list(map(float, coords))
    
    # Extract x-coordinates (even indices)
    x_coords = coords[::2]
    
    if not x_coords:
        return 0  # No coordinates found
    
    # Calculate width
    return max(x_coords) - min(x_coords)


def get_svg_path_height(path_d):
    """
    Calculate the height of an SVG path by extracting and analyzing its coordinates.

    Args:
        path_d (str): The `d` attribute of the SVG path.

    Returns:
        float: The height of the path (difference between max and min y-coordinates).
    """
    # Extract all coordinates using regex
    coords = re.findall(r"[-+]?\d*\.?\d+", path_d)
    coords = list(map(float, coords))
    
    # Extract y-coordinates (odd indices)
    y_coords = coords[1::2]
    
    if not y_coords:
        return 0  # No coordinates found
    
    # Calculate height
    return max(y_coords) - min(y_coords)


def main():
    parser = argparse.ArgumentParser(description='Generate keyboard layout')
    parser.add_argument('--stickers', action='store_true', help='Generate sticker layout (white on black)')
    parser.add_argument('--layout-card', action='store_true', help='Generate layout card (black on white)')
    args = parser.parse_args()

    if args.layout_card:
        config = LayoutConfig('layout-card')
        output_file = 'layout-card.svg'
    else:  # default to stickers mode
        config = LayoutConfig('stickers')
        output_file = 'layout.svg'

    ino_file = os.path.join(os.path.dirname(__file__), 'Preonic.ino')
    layers = parse_keymap(ino_file)
    svg = generate_svg(layers, config)
    
    with open(output_file, 'w') as f:
        f.write(svg)
    print(f'Generated layout at: {os.path.abspath(output_file)}')

if __name__ == '__main__':
    main()
