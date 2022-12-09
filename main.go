package main

import (
	"bufio"
	"fmt"
	"log"
	"os"
	"strings"
)

func add_controller_yaml(path string, template_controller_name string, new_controller_name string) (err_ error) {
	//open main yaml file, create a temp file and a new destination file

	f, err1 := os.Open(path + "fr3_ros.yaml")
	if err1 != nil {
		return err1
	}
	defer f.Close()
	f2, err2 := os.Create(path + "fr3_ros_temp.yaml")
	if err2 != nil {
		return err2
	}
	defer f2.Close()
	f3, err3 := os.Create(path + "fr3_ros_new.yaml")
	if err3 != nil {
		return err3
	}
	defer f3.Close()

	scanner := bufio.NewScanner(f) //create scanner for the main file
	reading_template_flag := 0

	for scanner.Scan() {
		if strings.Contains(scanner.Text(), template_controller_name) { // find where the template controller  is

			f2.WriteString(new_controller_name + "\n")
			reading_template_flag = 1
			continue
		}
		if reading_template_flag == 1 { //once we find it copy the template controller with new name into temp file
			f2.WriteString(scanner.Text() + "\n")
		}

		if len(scanner.Text()) == 0 && reading_template_flag == 1 {
			reading_template_flag = 0
			fmt.Println("dolol\n")
			break
		}

	}

	//close original and temp file, and reopen them in reading mode with new scanners
	f.Close()
	f2.Close()

	f_, err1 := os.Open(path + "fr3_ros.yaml")
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
	e := os.Remove(path + "fr3_ros_temp.yaml")
	if e != nil {
		log.Fatal(e)
	}
	return

}

func add_plugin(path string, controller_name string, controller_description string) (err_ error) {
	f, err1 := os.Open(path + "fr3_ros_plugin.xml")
	if err1 != nil {
		return err1
	}
	defer f.Close()
	f2, err2 := os.Create(path + "fr3_ros_plugin_new.xml")
	if err2 != nil {
		return err2
	}
	defer f2.Close()

	xml_template := "  <class name=\"fr3_ros/" + controller_name + "\" type=\"fr3_ros::" + controller_name + "\" base_class_type=\"controller_interface::ControllerBase\">\n    <description>\n" + "      " + controller_description + "\n    </description>\n  </class>"

	scanner := bufio.NewScanner(f)
	scanner.Scan()
	f2.WriteString(scanner.Text() + "\n")
	f2.WriteString(xml_template + "\n")

	for scanner.Scan() {
		f2.WriteString(scanner.Text() + "\n")
	}

	f.Close()
	f2.Close()
	return

}
func add_lib_CMAKE(path string, controller_name string) (err_ error) {
	f, err1 := os.Open(path + "CMakeLists.txt")
	if err1 != nil {
		return err1
	}
	defer f.Close()
	f2, err2 := os.Create(path + "CMakeLists_new.txt")
	if err2 != nil {
		return err2
	}
	defer f.Close()
	scanner := bufio.NewScanner(f)

	for scanner.Scan() {
		f2.WriteString("scanner.Text()+\n")
		if strings.Contains(scanner.Text(), "add_library") {
			// fmt.Println(scanner.Text())
			f2.WriteString("  src/" + controller_name + "\n")
		}
	}

	f.Close()
	f2.Close()
	return

}
func main() {

	// path_var := "/home/rum/fr3_ros/fr3_ros/"
	// lib_var := "sweeetlibraryname"
	// status := add_lib_CMAKE(path_var, lib_var)
	// fmt.Println(status)
	// path_var1 := "/home/rum/fr3_ros/fr3_ros/"
	// lib_var2 := "sweeetlibraryname"
	// status3 := add_plugin(path_var1, lib_var2, "new_desc")
	path_var5 := "/home/rum/fr3_ros/fr3_ros/config/"
	template_controller_name := "qp_controller:"
	new_controller_name := "joint_ppppp_controller"
	status4 := add_controller_yaml(path_var5, template_controller_name, new_controller_name)
	fmt.Println(status4)

}
