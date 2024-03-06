from dateutil import parser
import re
def txt_to_tum(input_file, output_file):
    with open(input_file, "r") as f_in, open(output_file, "w") as f_out:
        lines = reversed(f_in.readlines())
        for line in lines:
            timestamp_str = re.findall(r'timeStamp="(.*?)"', line)
            if len(timestamp_str) == 1:
                print (line)
                timestamp = parser.parse(timestamp_str[0]).strftime("%s.%f")
                coordinates = re.findall(r">(.+)<", line)[0].split()
                output_line = "{} {} {}\n".format(timestamp, ' '.join(coordinates), '0 0 0 1') 
                f_out.write(output_line)
if __name__ == '__main__': 

    files_to_process = ["20230731_1742_hybrid.xml" , "20230802_1547_grass.xml", \
                        "20230802_1854_dynamic.xml" , "20230802_1901_grass.xml", \
                        "20230802_2150_tunnel.xml", "20230802_2224_tunnel.xml"]
    
   ## for file_name in files_to_process:
        # input_file = file_name
        # output_file = file_name.replace(".xml", ".txt")
    file_name = "20230802_1917_grass.xml"    
    input_file = file_name
    output_file = file_name.replace(".xml", ".txt")
    txt_to_tum(input_file, output_file)
