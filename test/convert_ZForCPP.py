import csv
import os

def convert_to_gcode(input_file):
    gcode_commands = []

    
    # Open the CSV file and read its contents
    with open(input_file, mode='r') as file:
        reader = csv.reader(file)
        
        previousZ = 0
        skip = 0

        for row in reader:
            if skip >= 4:
                skip = 0
                continue
            elif skip > 0:
                skip += 1
                continue
            else:
                skip += 1
                
            # Skip empty rows or rows with insufficient columns
            if len(row) != 7:
                continue

            # Extract the values for X, Y, Z, U, V, W, and MoveTime
            X = row[0].strip()
            Y = row[1].strip()
            Z = row[2].strip()
            U = row[3].strip()
            V = row[4].strip()
            W = row[5].strip()
            MoveTime = row[6].strip()

            if previousZ == 0: previousZ = float(Z)

            dist = float(Z) - previousZ
            feedrate = abs((dist * 60000) / float(MoveTime))
            previousZ = float(Z)

            if feedrate == 0: feedrate = 50

            # Format the GCode command with the new constructor
            gcode_command = (
                f"    GCodeCommand('G', 6, "
                f"'X', {Z}f, 'Y', {Z}f, 'Z', {Z}f, 'U', {Z}f, 'V', {Z}f, 'W', {Z}f, 'F', {feedrate:.1f}f)"
            )
            
            gcode_commands.append(gcode_command)
    
    return gcode_commands

def save_gcode_to_file(gcode_commands, input_file):
    # Get the base name of the input file
    base_name, _ = os.path.splitext(input_file)
    
    # Create a new header file name
    output_file = f"{base_name}.h"

    # Write the GCode commands to the new header file
    with open(output_file, mode='w') as file:
        file.write("#pragma once\n\n")
        file.write("#include \"GCodeCommand.h\"\n\n")
        file.write("class GCodeCommandList {\n")
        file.write("public:\n")
        file.write(f"    static constexpr long commandCount = {len(gcode_commands)};\n\n")
        file.write("    static const GCodeCommand commandList[commandCount];\n\n")
        file.write("    static const GCodeCommand* GetCommand(long index) {\n")
        file.write("        if (index >= 0 && index < commandCount)\n")
        file.write("            return &commandList[index];\n")
        file.write("        else\n")
        file.write("            return nullptr;\n")
        file.write("    }\n")
        file.write("};\n\n")
        file.write(f"const GCodeCommand GCodeCommandList::commandList[commandCount] = {{\n")
        file.write(",\n".join(gcode_commands))
        file.write("\n};\n")
    
    print(f"GCode has been saved to {output_file}")

# Provide the path to the CSV file here
input_file = 'test/commands.csv'

# Convert to GCode
gcode_commands = convert_to_gcode(input_file)

# Save the GCode to a new file
save_gcode_to_file(gcode_commands, input_file)
