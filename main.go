package main

import (
	"bufio"
	"fmt"
	"log"
	"os"
	"strings"
)

func create_launch(path string, template_controller_name string, new_controller_name string) (err_ error) {
	//copy file first
	fmt.Println(path + template_controller_name + ".launch")
	source, err := os.Open(path + template_controller_name + ".launch")
	if err != nil {
		return err
	}
	defer source.Close()
	fmt.Println("Created and Modified all Files\n")

	destination, err := os.Create(path + new_controller_name + ".launch")
	if err != nil {
		return err
	}
	defer destination.Close()

	scanner := bufio.NewScanner(source) //create scanner for the new file

	for scanner.Scan() {

		if strings.Contains(scanner.Text(), template_controller_name) {
			destination.WriteString(strings.Replace(scanner.Text(), template_controller_name, new_controller_name, -1) + "\n")
		} else if !(strings.Contains(scanner.Text(), "</launch>")) {
			destination.WriteString(scanner.Text() + "\n")
		} else {
			destination.WriteString(scanner.Text())
		}

	}

	return err
}

func create_source(path string, template_controller_name string, new_controller_name string, template_code_name string, new_code_name string) (err_ error) {
	//copy file first
	source, err := os.Open(path + template_controller_name + ".cpp")
	if err != nil {
		return err
	}
	defer source.Close()

	destination, err := os.Create(path + new_controller_name + ".cpp")
	if err != nil {
		return err
	}
	defer destination.Close()

	scanner := bufio.NewScanner(source) //create scanner for the new file

	for scanner.Scan() {

		if strings.Contains(scanner.Text(), template_code_name) {
			destination.WriteString(strings.Replace(scanner.Text(), template_code_name, new_code_name, -1) + "\n")
		} else if !(strings.Contains(scanner.Text(), "PLUGINLIB_EXPORT_CLASS")) {
			destination.WriteString(scanner.Text() + "\n")
		} else {
			destination.WriteString(scanner.Text())
		}

	}

	return err
}

func create_header(path string, template_controller_name string, new_controller_name string, template_code_name string, new_code_name string) (err_ error) {
	//copy file first
	source, err := os.Open(path + template_controller_name + ".h")
	if err != nil {
		return err
	}
	defer source.Close()

	destination, err := os.Create(path + new_controller_name + ".h")
	if err != nil {
		return err
	}
	defer destination.Close()

	scanner := bufio.NewScanner(source) //create scanner for the new file

	for scanner.Scan() {

		if strings.Contains(scanner.Text(), template_code_name) {
			destination.WriteString(strings.Replace(scanner.Text(), template_code_name, new_code_name, -1) + "\n")
		} else if !(strings.Contains(scanner.Text(), "// namespace fr3_ros")) {
			destination.WriteString(scanner.Text() + "\n")
		} else {
			destination.WriteString(scanner.Text())
		}

	}

	return err
}

func add_controller_yaml(path string, template_controller_name string, new_controller_name string) (err_ error) {
	//open main yaml file, create a temp file and a new destination file

	e := os.Rename(path+"fr3_ros.yaml", path+"fr3_ros.yaml"+".bak")
	if e != nil {
		log.Fatal(e)
	}

	f, err1 := os.Open(path + "fr3_ros.yaml.bak")
	if err1 != nil {
		return err1
	}
	defer f.Close()

	f2, err2 := os.Create(path + "fr3_ros_temp.yaml")
	if err2 != nil {
		return err2
	}
	defer f2.Close()

	f3, err3 := os.Create(path + "fr3_ros.yaml")
	if err3 != nil {
		return err3
	}
	defer f3.Close()

	scanner := bufio.NewScanner(f) //create scanner for the main file
	reading_template_flag := 0
	second_line_flag := 0

	for scanner.Scan() {
		if strings.Contains(scanner.Text(), template_controller_name) { // find where the template controller  is

			f2.WriteString(new_controller_name + ":\n")
			f2.WriteString("    type: fr3_ros/" + new_controller_name + "\n")
			reading_template_flag = 1
			second_line_flag = 1
			continue
		}
		if reading_template_flag == 1 { //once we find it copy the template controller with new name into temp file
			if second_line_flag == 1 {
				second_line_flag = 0
				continue
			}
			f2.WriteString(scanner.Text() + "\n")
		}

		if len(scanner.Text()) == 0 && reading_template_flag == 1 {
			reading_template_flag = 0
			break
		}

	}

	//close original and temp file, and reopen them in reading mode with new scanners
	f.Close()
	f2.Close()

	f_, err1 := os.Open(path + "fr3_ros.yaml.bak")
	if err1 != nil {
		return err1
	}
	defer f_.Close()

	f2_, err2 := os.Open(path + "fr3_ros_temp.yaml")
	if err2 != nil {
		return err2
	}
	defer f2_.Close()

	scanner_ := bufio.NewScanner(f_)
	scanner2 := bufio.NewScanner(f2_)

	//copy original into new file
	for scanner_.Scan() {
		f3.WriteString(scanner_.Text() + "\n")
	}
	//and append the template controller with new name to the end of the new file
	for scanner2.Scan() {
		if len(scanner2.Text()) == 0 {
			break
		}
		f3.WriteString("\n" + scanner2.Text())
	}

	//close all files
	f_.Close()
	f2_.Close()
	f3.Close()

	//delete temp file
	e = os.Remove(path + "fr3_ros_temp.yaml")
	if e != nil {
		log.Fatal(e)
	}
	return

}

func add_plugin(path string, new_controller_name string, controller_description string) (err_ error) {
	e := os.Rename(path+"fr3_ros_plugin.xml", path+"fr3_ros_plugin.xml"+".bak")
	if e != nil {
		log.Fatal(e)
	}

	f, err1 := os.Open(path + "fr3_ros_plugin.xml.bak")
	if err1 != nil {
		return err1
	}
	defer f.Close()
	f2, err2 := os.Create(path + "fr3_ros_plugin.xml")
	if err2 != nil {
		return err2
	}
	defer f2.Close()

	xml_template := "  <class name=\"fr3_ros/" + new_controller_name + "\" type=\"fr3_ros::" + new_controller_name + "\" base_class_type=\"controller_interface::ControllerBase\">\n    <description>\n" + "      " + controller_description + "\n    </description>\n  </class>"

	scanner := bufio.NewScanner(f)
	scanner.Scan()
	f2.WriteString(scanner.Text() + "\n")

	for scanner.Scan() {
		if strings.Contains(scanner.Text(), "</library>") {
			f2.WriteString(xml_template + "\n")
		}
		f2.WriteString(scanner.Text() + "\n")
	}

	f.Close()
	f2.Close()
	return

}
func add_lib_CMAKE(path string, controller_name string) (err_ error) {

	e := os.Rename(path+"CMakeLists.txt", path+"CMakeLists.txt"+".bak")
	if e != nil {
		log.Fatal(e)
	}

	f2, err2 := os.Create(path + "CMakeLists.txt")
	if err2 != nil {
		return err2
	}

	source, err := os.Open(path + "CMakeLists.txt.bak")
	if err != nil {
		return err
	}

	scanner := bufio.NewScanner(source)

	for scanner.Scan() {
		f2.WriteString(scanner.Text() + "\n")
		if strings.Contains(scanner.Text(), "add_library") {
			// fmt.Println(scanner.Text())
			f2.WriteString("  src/" + controller_name + "\n")
		}
	}

	source.Close()
	f2.Close()

	return

}
func main() {

	cwd_, err := os.Getwd()
	if err != nil {
		log.Println(err)
	}
	fmt.Println(cwd_)

	path_CMake := cwd_ + "/fr3_ros/"
	path_plugin := path_CMake
	path_yaml := cwd_ + "/fr3_ros/config/"
	path_header := cwd_ + "/fr3_ros/include/fr3_ros/"
	path_source := cwd_ + "/fr3_ros/src/"
	path_launch := cwd_ + "/fr3_ros/launch/"

	template_controller_name := "joint_pd_controller"
	template_controller_code_name := "JointPDController"

	new_controller_name := "bololo_controller"
	new_controller_code_name := "JJJJOPController"

	new_controller_description := "This is an overpowered controller"

	_ = add_lib_CMAKE(path_CMake, new_controller_name)
	_ = add_plugin(path_plugin, new_controller_name, new_controller_description)
	_ = add_controller_yaml(path_yaml, template_controller_name, new_controller_name)
	_ = create_header(path_header, template_controller_name, new_controller_name, template_controller_code_name, new_controller_code_name)
	_ = create_source(path_source, template_controller_name, new_controller_name, template_controller_code_name, new_controller_code_name)
	_ = create_launch(path_launch, template_controller_name, new_controller_name)

	fmt.Println("Created and Modified all Files\n")
}
