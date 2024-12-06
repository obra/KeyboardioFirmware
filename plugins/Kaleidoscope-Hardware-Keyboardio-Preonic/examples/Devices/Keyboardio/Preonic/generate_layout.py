#!/usr/bin/env python3

import re
import os

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
    svg = '''<svg width="14" height="14" fill="none" stroke="currentColor" stroke-width="2" version="1.1" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
 <path d="m6.0006 9.9714v10c0 4 12 4 12 0v-10s0-4-6-4c-6 0-6 4-6 4z"/>
 <path d="m11.656 6.8328s0-3 1-4 2-2-1-2" stroke-width="1.5"/>

'''

    if button is None:
        # Add curvy tail for movement mouse
        return svg + '''
</svg>'''
    else:
        # Add button indicator (width matches divider lines)
        button_x = {'left': 8, 'middle': 11.25, 'right': 14.5}[button]
        svg += f'''
    <rect x="{button_x}" y="8" width="1.5" height="5" fill="currentColor" opacity="1" clip-path="url(#mouseClip)"/>'''
        return svg + '''
</svg>'''

def get_bluetooth_icon():
    return '''<svg viewBox="0 0 24 24" width="14" height="14" stroke="currentColor" stroke-width="2" fill="none" style="margin-right:-0.25em">
    <path d="M6,7 L18,17 L12,22 L12,2 L18,7 L6,17" rx="1"/>
</svg>'''

def get_bluetooth_off_icon():
    return get_bluetooth_icon() + 'off'

def get_volume_up_icon():
    return '''<svg viewBox="0 0 24 24" width="14" height="14" stroke="currentColor" stroke-width="2" fill="none">
    <path d="M12,7 L12,17 L7,13 L4,13 L4,11 L7,11 L12,7 Z" rx="1"/>
    <path d="M15,9 C17,11 17,13 15,15" rx="0.5"/>
    <path d="M17,7 C20,10 20,14 17,17" rx="0.5"/>
</svg>'''

def get_volume_down_icon():
    return '''<svg viewBox="0 0 24 24" width="14" height="14" stroke="currentColor" stroke-width="2" fill="none">
    <path d="M12,7 L12,17 L7,13 L4,13 L4,11 L7,11 L12,7 Z" rx="1"/>
    <path d="M15,9 C17,11 17,13 15,15" rx="0.5"/>
</svg>'''

def get_mute_icon():
    return '''<svg viewBox="0 0 24 24" width="14" height="14" stroke="currentColor" stroke-width="2" fill="none">
    <path d="M12,7 L12,17 L7,13 L4,13 L4,11 L7,11 L12,7 Z" rx="1"/>
    <line x1="15" y1="9" x2="20" y2="15"/>
    <line x1="20" y1="9" x2="15" y2="15"/>
</svg>'''

def get_prev_track_icon():
    return '''<svg width="14" height="14" fill="none" stroke="currentColor" stroke-width="2" version="1.1" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
 <path d="m6.2286 6v12" stroke-linecap="round"/>
 <path rx="1" d="m18.8 6-9 6 9 6z" stroke-linejoin="round"/>
</svg>'''

def get_next_track_icon():
    return '''<svg width="14" height="14" fill="none" stroke="currentColor" stroke-width="2" version="1.1" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
 <path d="m18.8 6v12" stroke-linecap="round"/>
 <path rx="1" d="m6.2286 6 9 6-9 6z" stroke-linejoin="round"/>
</svg>'''

def get_led_effect_icon():
    return '''<svg viewBox="0 0 24 24" width="14" height="14" stroke="currentColor" stroke-width="2" fill="none" style="margin-right:-0.25em">
    <circle cx="12" cy="12" r="5"/>
    <path d="M12,3 L12,7 M21,12 L17,12 M12,21 L12,17 M3,12 L7,12"/>
    <path d="M18.4,5.6 L15.6,8.4 M18.4,18.4 L15.6,15.6 M5.6,18.4 L8.4,15.6 M5.6,5.6 L8.4,8.4" stroke-width="1.5"/>
</svg>'''

def get_play_pause_icon():
    return '''<svg width="20" height="20" fill="none" stroke="white" stroke-width="2" version="1.1" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
 <g stroke-linecap="round" stroke-linejoin="round">
  <path rx="1" d="m7 6v12l7-6z"/>
  <path d="m16.2 6v12m3-12v12"/>
 </g>
</svg>
'''

def get_volume_arrow(direction):
    return f'''<svg viewBox="0 0 24 24" width="10" height="10" stroke="currentColor" stroke-width="2" fill="none">
    <path d="M12,{20 if direction == 'down' else 4} L8,{14 if direction == 'down' else 10} L16,{14 if direction == 'down' else 10} Z"/>
</svg>'''

def get_keyclick_icon():
    return '''<svg viewBox="0 0 24 24" width="14" height="14" stroke="currentColor" stroke-width="2" fill="none">
    <rect x="6" y="8" width="8" height="8" rx="1"/>
    <path d="M16,10 C18,12 18,12 16,14" stroke-width="1.5"/>
    <path d="M18,8 C21,12 21,12 18,16" stroke-width="1.5"/>
</svg>'''

def get_warp_icon(quadrant):
    # quadrant can be 'nw', 'ne', 'sw', 'se'
    fill = {
        'nw': 'M4,4 L12,4 L12,12 L4,12 Z',
        'ne': 'M12,4 L20,4 L20,12 L12,12 Z',
        'sw': 'M4,12 L12,12 L12,20 L4,20 Z',
        'se': 'M12,12 L20,12 L20,20 L12,20 Z'
    }[quadrant]
    return get_mouse_icon() + f'''<svg viewBox="0 0 24 24" width="14" height="14" stroke="currentColor" stroke-width="2" fill="none">
    <rect x="2" y="2" width="20" height="20"  rx="1"/>
    <path d="{fill}" fill="currentColor" opacity="1"  rx="1" />
</svg>'''

def get_warp_end_icon():
    return get_mouse_icon() + '''<svg viewBox="0 0 24 24" width="14" height="14" stroke="currentColor" stroke-width="2" fill="none">
    <rect x="4" y="4" width="16" height="16"/>
</svg>'''

def get_usb_icon():
    return '''<svg viewBox="0 0 24 24" width="14" height="14" stroke="currentColor" stroke-width="2" fill="none" style="margin-right:-0.25em">
    <path d="M8,4 L16,4 L16,8 L18,8 L18,16 L14,16 L14,20 L10,20 L10,16 L6,16 L6,8 L8,8 Z" rx="1"/>
    <path d="M10,4 L10,8 M14,4 L14,8" stroke-width="1"/>
</svg>'''

def clean_key_name(key):
    if not key or key == '___' or key == 'XXX':
        return ''
    
    # Remove any extra spaces
    key = key.strip()
    
    # Special handling for layer keys - return tuple of (text, css_class)
    mo_match = re.match(r'MO\((\w+)\)', key.replace(' ', ''))
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
        'Key_mouseUp': get_mouse_icon() + '↑',
        'Key_mouseDown': get_mouse_icon() + '↓',
        'Key_mouseLeft': get_mouse_icon() + '←',
        'Key_mouseRight': get_mouse_icon() + '→',
        'Key_mouseL': get_mouse_icon() + '←',
        'Key_mouseDn': get_mouse_icon() + '↓',
        'Key_mouseR': get_mouse_icon() + '→',
        'Key_LEDEffectNext': get_led_effect_icon(),
        'M(MACRO_BT_OFF)': get_usb_icon(),
        'M(MACRO_BT_SELECT_1)': get_bluetooth_icon() + '①',
        'M(MACRO_BT_SELECT_2)': get_bluetooth_icon() + '②',
        'M(MACRO_BT_SELECT_3)': get_bluetooth_icon() + '③',
        'M(MACRO_BT_PAIR)': get_bluetooth_icon() + '⟲',
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

def generate_html(layers):
    def render_key(idx, row, col, is_space_key=False):
        # Check if this is the volume knob
        if row == 0 and col == 11:
            return f'''            <div class="key row-{row} col-{col}">
                <div class="volume-text">
                    <svg viewBox="0 0 60 20">
                        <path id="curve" fill="none" d="M10,19 A30,30 0 0,1 50,19" />
                        <text fill="#888" font-size="8">
                            <textPath href="#curve" startOffset="50%" text-anchor="middle">
                                -VOL+
                            </textPath>
                        </text>
                    </svg>
                </div>
                <div class="play-pause">{get_play_pause_icon()}</div>
            </div>
'''
        # Skip rendering the first two keys in row 0 (they're now part of the knob display)
        if row == 0 and (col == 7 or col == 8):
            return ''
        if is_space_key:
            return f'''            <div class="key double-width row-{row} col-{col}">
                <span class="legend main">Space</span>
                <span class="legend lower">Bksp</span>
            </div>
'''
        qwerty_key = layers['QWERTY'][idx] if idx < len(layers['QWERTY']) else ''
        lower_key = layers['LOWER'][idx] if idx < len(layers.get('LOWER', [])) else ''
        upper_key = layers['UPPER'][idx] if idx < len(layers.get('UPPER', [])) else ''
        fun_key = layers['FUN'][idx] if idx < len(layers.get('FUN', [])) else ''
        
        # Clean up key names
        qwerty_key = clean_key_name(qwerty_key)
        lower_key = clean_key_name(lower_key)
        upper_key = clean_key_name(upper_key)
        fun_key = clean_key_name(fun_key)

        html = f'''            <div class="key row-{row} col-{col}">
'''
        # Modified key generation to handle layer keys
        if qwerty_key:
            if isinstance(qwerty_key, tuple):
                text, css_class = qwerty_key
                html += f'''                <span class="legend main {css_class}">{text}</span>
'''
            elif qwerty_key != '___':
                html += f'''                <span class="legend main">{qwerty_key}</span>
'''
        if lower_key and lower_key != '___':
            html += f'''                <span class="legend lower">{lower_key}</span>
'''
        if upper_key and upper_key != '___':
            html += f'''                <span class="legend upper">{upper_key}</span>
'''
        if fun_key and fun_key != '___':
            html += f'''                <span class="legend fun">{fun_key}</span>
'''
        html += '''            </div>
'''
        return html

    html_template = '''<!DOCTYPE html>
<html>
<head>
    <title>Preonic Keyboard Layout</title>
    <style>
        body {
            font-family: "Gorton Perfected", -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Arial, sans-serif;
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
            margin: 0;
            background: #000000;
        }

        .keyboard {
            background: #111111;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 4px 6px rgba(255, 255, 255, 0.1);
            position: relative;
        }

        .butterfly-logo {
            position: absolute;
            top: 16px;  /* Original center (40px) - new_height/2 (24px) */
            left: 58px;  /* Original center (82px) - new_width/2 (24px) */
            width: 48px;  /* 60px * 0.8 = 48px */
            height: 48px;  /* 60px * 0.8 = 48px */
            z-index: 1;
        }

        .butterfly-logo svg {
            width: 100%;
            height: 100%;
        }

        .butterfly-logo path {
            fill: #ffffff;
            stroke: #333333;
            stroke-width: 3;
        }

        .key-row {
            display: flex;
            gap: 4px;
            margin-bottom: 4px;
            position: relative;
        }

        .top-row {
            justify-content: flex-end;
        }

        .key {
            width: 60px;
            height: 60px;
            background: #222222;
            border: 1px solid #333333;
            border-radius: 4px;
            position: relative;
            box-shadow: 0 2px 4px rgba(0, 0, 0, 0.5);
            box-sizing: border-box;
        }

        .key.double-width {
            width: 124px;  /* (60px * 2) + 4px = two keys plus one interkey space */
        }

        .legend {
            position: absolute;
            font-size: 12px;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 1px;
        }

        .legend svg {
            vertical-align: middle;
            margin-right: 1px;
        }

        .main {
            bottom: 4px;
            left: 4px;
            color: #ffffff;
        }

        .lower {
            bottom: 4px;
            right: 4px;
            color: #66b3ff;  /* Brighter blue for dark background */
        }

        .upper {
            top: 4px;
            right: 4px;
            color: #ff944d;  /* Brighter orange for dark background */
        }

        .fun {
            top: 4px;
            left: 4px;
            color: #66ff66;  /* Brighter green for dark background */
        }

        /* Special handling for layer keys */
        .layer-lower {
            color: #66b3ff;  /* Brighter blue for dark background */
        }

        .layer-upper {
            color: #ff944d;  /* Brighter orange for dark background */
        }

        .layer-fun {
            color: #66ff66;  /* Brighter green for dark background */
        }

        .key.row-0.col-11 {
            border-radius: 50%;
            background: linear-gradient(135deg, #2a2a2a 0%, #222222 100%);
            border: 2px solid #333333;
            position: relative;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            padding: 8px 0;
            overflow: visible;
        }

        .key.row-0.col-11 .volume-text {
            position: absolute;
            top: -2px;
            left: 0;
            width: 100%;
            height: 20px;
        }

        .key.row-0.col-11 .volume-text svg {
            width: 100%;
            height: 100%;
        }

        .key.row-0.col-11 .volume-text text {
            font-size: 8px;
            font-family: monospace;
            letter-spacing: 0px;
        }

        .key.row-0.col-11 .play-pause {
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
        }


        .key.double-width {
            width: 124px;  /* (60px * 2) + 4px = two keys plus one interkey space */
        }

        .butterfly-logo {
            position: absolute;
            top: 16px;  /* Original center (40px) - new_height/2 (24px) */
            left: 58px;  /* Original center (82px) - new_width/2 (24px) */
            width: 48px;  /* 60px * 0.8 = 48px */
            height: 48px;  /* 60px * 0.8 = 48px */
            z-index: 1;
        }

        .butterfly-logo svg {
            width: 100%;
            height: 100%;
        }

        .butterfly-logo path {
            fill: #ffffff;
            stroke: #333333;
            stroke-width: 3;
        }
    </style>
</head>
<body>
    <div class="keyboard">
        <div class="butterfly-logo">
            <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 80 851 552">
                <g >
                    <path d="M422.6 261.1c-21.2 0-38.7 35-38.7 95.2 0 60.3 15.6 188.3 38.7 188.3 23.1 0 38.7-128 38.7-188.3 0-60.3-17.5-95.2-38.7-95.2"/>
                    <path transform="matrix(.8869 -.4619 .4619 .8869 -55.651 199.1881)" d="M369.5 176.6h19v73.4h-19z"/>
                    <path transform="matrix(.4618 -.887 .887 .4618 61.4743 527.8238)" d="M429 203.8h73.4v19H429z"/>
                    <path d="M58 90.4C17.2 80.7-6.2 105.3 1.2 154c15.5 101.9 52 164.5 129.9 183.5 59.3 14.4 221.8 20 221.8 20-33.3-162.4-195.1-243.4-294.9-267.1"/>
                    <path d="M355 389.5s-71.1-5.6-119-1.6c-61 5.1-98 25.6-98 96.2 0 103.5 99.5 150.2 150.4 147.5 20.9-1.1 56.9-44.1 69.5-77.5 8.3-21.9 11.9-42.3 8.9-73.9-1.8-19.4-11.8-90.7-11.8-90.7"/>
                    <path d="M786.7 90.4c-99.8 23.7-261.7 104.7-295 267.1 0 0 162.6-5.6 221.8-20 77.9-19 114.4-81.6 129.9-183.5 7.5-48.7-15.9-73.3-56.7-63.6"/>
                    <path d="M608.7 387.9c-47.8-4-119 1.6-119 1.6s-10 71.4-11.8 90.7c-3 31.6.7 51.9 8.9 73.9 12.6 33.4 48.5 76.4 69.5 77.5 50.8 2.7 150.4-44 150.4-147.5 0-70.6-37-91.1-98-96.2"/>
                </g>
            </svg>
        </div>
        <div class="key-row top-row">
'''
    for i in range(5):
        html_template += render_key(i, 0, i + 7)
    html_template += '        </div>\n\n'
    
    # Add the main 5x12 grid
    start_idx = 5  # Start after the first row (5 keys)
    for row in range(5):  # 5 rows
        html_template += '''        <div class="key-row">
'''
        for col in range(12):  # 12 columns
            idx = start_idx + row * 12 + col
            is_space_key = (row == 4 and (col == 5 or col == 6))  # Space key is in row 4 (5th row)
            if is_space_key and col == 5:  # Only add the key once
                html_template += render_key(idx, row + 1, col, is_space_key=True)
                continue
            elif is_space_key and col == 6:
                continue  # Skip the second key of the space bar
            html_template += render_key(idx, row + 1, col)
        html_template += '''        </div>
'''
    
    html_template += '''    </div>
</body>
</html>'''
    
    return html_template

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ino_file = os.path.join(script_dir, 'Preonic.ino')
    html_file = os.path.join(script_dir, 'layout.html')
    
    layers = parse_keymap(ino_file)
    if not layers:
        print("Error: Could not parse keymap from .ino file")
        return
    
    html_content = generate_html(layers)
    with open(html_file, 'w') as f:
        f.write(html_content)
    
    print(f"Generated layout at: {html_file}")

if __name__ == '__main__':
    main()
