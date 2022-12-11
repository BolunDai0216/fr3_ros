package main

// A simple program demonstrating the text input component from the Bubbles
// component library.

import (
	"bufio"
	"errors"
	"fmt"
	"io/ioutil"
	"log"
	"os"
	"strings"
	"unicode/utf8"

	"github.com/charmbracelet/bubbles/textinput"
	tea "github.com/charmbracelet/bubbletea"
	"github.com/charmbracelet/lipgloss"
	"golang.org/x/text/cases"
	"golang.org/x/text/language"
)

func main() {
	p := tea.NewProgram(initialModel())
	if _, err := p.Run(); err != nil {
		log.Fatal(err)
	}

}

type (
	errMsg error
)

type model struct {
	textInput              textinput.Model
	controller_description textinput.Model
	custom_cpp_name        textinput.Model
	custom_class_name      textinput.Model
	template_file_names    []string

	template_controller textinput.Model
	err                 error
	state               string
	cpp_names           []string
	cursor              int
	cursor2             int
	curentdir           string
	path_CMake          string
	path_plugin         string
	path_yaml           string
	path_header         string
	path_source         string
	path_launch         string
	path_backup         string
}

func initialModel() model {
	ti := textinput.New()
	ti.Placeholder = "quadratic_program_controller"
	ti.Focus()
	ti.CharLimit = 156
	ti.Width = 100
	ti.TextStyle = lipgloss.NewStyle().Foreground(lipgloss.Color("87"))
	ti.SetValue(ti.Placeholder)

	ti1 := textinput.New()
	ti1.Placeholder = "This is an example description for the controller you would like to make."
	// ti.Focus()
	ti1.CharLimit = 250
	ti1.Width = 100
	ti1.TextStyle = lipgloss.NewStyle().Foreground(lipgloss.Color("87"))
	ti1.SetValue(ti1.Placeholder)

	ti3 := textinput.New()
	ti3.Placeholder = "CustomCppControllerName"
	// ti3.Focus()
	ti3.CharLimit = 156
	ti3.Width = 100
	ti3.TextStyle = lipgloss.NewStyle().Foreground(lipgloss.Color("87"))
	ti3.SetValue(ti3.Placeholder)

	ti4 := textinput.New()
	ti4.Placeholder = "CustomCppControllerName"
	// ti3.Focus()
	ti4.CharLimit = 156
	ti4.Width = 100
	ti4.TextStyle = lipgloss.NewStyle().Foreground(lipgloss.Color("87"))
	ti4.SetValue(ti4.Placeholder)

	cwd_, err := os.Getwd()
	if err != nil {
		log.Println(err)
	}

	var temp_arr [2]string
	temp_arr[0] = ""
	temp_arr[1] = "Custom"
	return model{
		textInput:              ti,
		controller_description: ti1,
		custom_cpp_name:        ti3,
		custom_class_name:      ti4,
		err:                    nil,
		state:                  "input_controller_name",
		template_file_names:    temp_arr[:],
		cpp_names:              temp_arr[:],
		curentdir:              cwd_,
		cursor:                 0,
		path_CMake:             cwd_ + "/fr3_ros/",
		path_plugin:            cwd_ + "/fr3_ros/",
		path_yaml:              cwd_ + "/fr3_ros/config/",
		path_header:            cwd_ + "/fr3_ros/include/fr3_ros/",
		path_source:            cwd_ + "/fr3_ros/src/",
		path_launch:            cwd_ + "/fr3_ros/launch/",
		path_backup:            cwd_ + "/backup/",
	}
}

func getClassName(template_controller_name string, path string) (class_name string, err_ error) {
	var temp_arr []string
	var temp string
	// fmt.Println(path + template_controller_name + ".h")
	source1, err := os.Open(path + template_controller_name + ".h")
	if err != nil {
		return "s", err
	}

	scanner := bufio.NewScanner(source1) //create scanner for the new file

	for scanner.Scan() {

		if strings.Contains(scanner.Text(), "class") {
			// fmt.Println(scanner.Text())

			temp = scanner.Text()
			break
		}

	}

	temp_arr = strings.Fields(temp)
	// fmt.Println(temp_arr)

	source1.Close()
	return temp_arr[1], err

}

func populateTemplateChoices(path string) (choices []string) {

	blacklist := [...]string{"cbf_utils", "controller_utils", "pinocchio_utils", "traj_gen", "visualization_utils"}
	skip_flag := 0
	files, err := ioutil.ReadDir(path)
	if err != nil {
		log.Fatal(err)
	}
	for _, files := range files {
		// fmt.Println()
		skip_flag = 0

		for j, _ := range blacklist {
			if strings.Contains(files.Name(), blacklist[j]) {
				skip_flag = 1
				break
			}
		}
		if skip_flag == 1 {
			continue
		}
		choices = append(choices, strings.Replace(files.Name(), ".cpp", "", -1))
	}
	return choices
}

func populateChoices(user_input_controller_name string) (choices []string) {

	// choices = append(choices, user_input_controller_name)
	choices = append(choices, strings.Replace(cases.Title(language.Und, cases.NoLower).String(strings.Replace(user_input_controller_name, "_", " ", -1)), " ", "", -1))
	// temp_string_arr := strings.Split(user_input_controller_name, "_")
	// temp_string := strings.ToUpper(temp_string_arr[0])
	// if len(temp_string) > 1 {
	// 	for i := 1; i < len(temp_string_arr); i++ {
	// 		temp_string += cases.Title(language.Und, cases.NoLower).String(temp_string_arr[i])
	// 	}
	// }
	// choices = append(choices, temp_string)
	choices = append(choices, "Custom")
	return choices

}

func (m model) Init() tea.Cmd {
	return textinput.Blink
}

func (m model) Update(msg tea.Msg) (tea.Model, tea.Cmd) {
	var cmd tea.Cmd

	switch msg := msg.(type) {
	case tea.KeyMsg:
		switch msg.Type {
		case tea.KeyCtrlC, tea.KeyEsc:
			return m, tea.Quit
		case tea.KeyEnter:
			switch m.state {
			case "input_controller_name":
				m.state = "input_class_name"
				m.textInput.Blur()
				m.custom_class_name.Focus()
				m.cpp_names = populateChoices(m.textInput.Value())
				m.custom_class_name.SetValue(m.cpp_names[0])
			case "input_class_name":
				m.state = "input_description"
				m.cpp_names[0] = m.custom_class_name.Value()
				m.custom_class_name.Blur()
				m.controller_description.Focus()
			case "input_description":
				m.state = "choose_template"
				m.template_file_names = populateTemplateChoices(m.path_source)
				m.controller_description.Blur()
			case "choose_template":
				m.state = "final_confirmation"

			case "final_confirmation":
				if _, err := os.Stat("backup"); errors.Is(err, os.ErrNotExist) {
					err := os.Mkdir("backup", os.ModePerm)
					if err != nil {
						log.Println(err)
					}
				}
				m.state = "finished"
				template_controller_code_name, _ := getClassName(m.template_file_names[m.cursor], m.path_header)
				_ = add_lib_CMAKE(m.path_CMake, m.path_backup, m.textInput.Value())
				_ = add_plugin(m.path_plugin, m.path_backup, m.textInput.Value(), m.controller_description.Value())
				_ = add_controller_yaml(m.path_yaml, m.path_backup, m.template_file_names[m.cursor], m.textInput.Value())
				_ = create_header(m.path_header, m.template_file_names[m.cursor], m.textInput.Value(), template_controller_code_name, m.custom_class_name.Value())
				_ = create_source(m.path_source, m.template_file_names[m.cursor], m.textInput.Value(), template_controller_code_name, m.custom_class_name.Value())
				_ = create_launch(m.path_launch, m.template_file_names[m.cursor], m.textInput.Value())
			case "finished":
				return m, tea.Quit
			}
		default:
			switch m.state {
			case "input_controller_name":
				m.textInput, cmd = m.textInput.Update(msg)
			case "input_class_name":
				m.custom_class_name, cmd = m.custom_class_name.Update(msg)
			case "input_description":
				m.controller_description, cmd = m.controller_description.Update(msg)
			case "choose_template":
				switch msg.String() {
				case "up", "k":
					if m.cursor2 > 0 {
						m.cursor2--
					}

				// The "down" and "j" keys move the cursor down
				case "down", "j":
					if m.cursor2 < len(m.template_file_names)-1 {
						m.cursor2++
					}

				}
				if m.cursor2 < (len(m.template_file_names)+1)/2 {
					m.cursor = m.cursor2 * 2
				} else {
					m.cursor = (m.cursor2-((len(m.template_file_names)-1)/2))*2 - 1
				}
			case "final_step":

			}

		}

	// We handle errors just like any other message
	case errMsg:
		m.err = msg
		return m, nil
	}

	return m, cmd
}

func (m model) View() string {
	s := ""

	x := [...]string{"You chose your controller name as > \"" + m.textInput.Value() + "\"\n",
		"You chose your controller .cpp name as > \"" + m.custom_class_name.Value() + ".cpp\"\n",
		"Your controller description is> \n   \"" + m.controller_description.Value() + "\"\n\n",
		"You chose the template as > \"" + m.template_file_names[m.cursor] + "\"\n\n"}
	tail := "\n\n[Enter] to continue, [Esc] to quit)"
	switch m.state {
	case "finished":
		s = "Success! Press [Enter] button to quit\n"
	case "final_confirmation":

		s = ""
		for i := 0; i < 4; i++ {
			s += x[i]
		}
		s += "If all of this information is correct, press [Enter] to generate starter files for your new controller! \nOtherwise press [Esc] to try again"
		s += tail
	case "choose_template":
		prev_len := 0
		temp := ""
		s = ""
		for i := 0; i < 4; i++ {
			s += x[i]
		}
		s += "Finally, please choose the template controller you'd like to use as a starting point for your new controller>\n\n"

		for i, choice := range m.template_file_names {
			cursor2 := " "
			// if m.cursor2 == i {
			// 	cursor2 = ">"
			// }

			checked := " "

			// if i < (len(m.template_file_names)+1)/2 {
			if m.cursor2 == i/2.0 && i%2 == 0 {
				cursor2 = ">"
				// m.cursor = i
			} else {
				if i%2 == 1 && m.cursor2 == (len(m.template_file_names)+1)/2+(i-1)/2 {
					cursor2 = ">"
					// m.cursor = i
				}
			}

			// }
			if i%2 == 0 {

				temp = fmt.Sprintf("%s [%s] %s.cpp", cursor2, checked, choice)
				s += temp
				prev_len = utf8.RuneCountInString(temp)
			} else {

				// if m.cursor2 == (len(m.template_file_names)+1)/2+i {
				// 	cursor2 = ">"
				// }
				s += strings.Repeat(" ", (50-prev_len)) + fmt.Sprintf("%s [%s] %s.cpp\n", cursor2, checked, choice)
			}

		}
		s += "\n\n\n(use [Up] and [Down] arrows to select, [Enter] to select, [Esc] to quit)\n"

	case "input_description":
		s = ""
		for i := 0; i < 3; i++ {
			s += x[i]
		}

		s += fmt.Sprintf(
			"\n%s%s",
			"Almost there! Now please provide a one line description of your new controller\n\n",
			m.controller_description.View())
		s += tail

	case "input_class_name":
		s = ""
		for i := 0; i < 2; i++ {
			s += x[i]
		}

		s += fmt.Sprintf("\n%s%s",
			"Press [Enter] to accept the following class name for your controller, else change it:\n\n",
			m.custom_class_name.View(),
		)
		s += tail

	case "input_controller_name":
		s += fmt.Sprintf(
			"You're about to create a new controller for the FR3 Franka Research Robot\nWhat would you like to name your controller?\n%s\n\n",
			m.textInput.View())
		s += tail
	}
	return s
}

func create_launch(path string, template_controller_name string, new_controller_name string) (err_ error) {
	//copy file first
	// fmt.Println(path + template_controller_name + ".launch")
	source, err := os.Open(path + template_controller_name + ".launch")
	if err != nil {
		return err
	}
	defer source.Close()
	// fmt.Println("Created and Modified all Files\n")

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
		} else if strings.Contains(scanner.Text(), template_controller_name) {
			destination.WriteString(strings.Replace(scanner.Text(), template_controller_name, new_controller_name, -1) + "\n")
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
		} else if strings.Contains(scanner.Text(), template_controller_name) {
			destination.WriteString(strings.Replace(scanner.Text(), template_controller_name, new_controller_name, -1) + "\n")
		} else if !(strings.Contains(scanner.Text(), "// namespace fr3_ros")) {
			destination.WriteString(scanner.Text() + "\n")
		} else {
			destination.WriteString(scanner.Text())
		}

	}

	return err
}

func add_controller_yaml(path string, backup_path string, template_controller_name string, new_controller_name string) (err_ error) {
	//open main yaml file, create a temp file and a new destination file

	e := os.Rename(path+"fr3_ros.yaml", path+"fr3_ros.yaml"+".backup")
	if e != nil {
		log.Fatal(e)
	}

	f, err1 := os.Open(path + "fr3_ros.yaml.backup")
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

	f_, err1 := os.Open(path + "fr3_ros.yaml.backup")
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

	err := os.Rename(path+"fr3_ros.yaml.backup", backup_path+"fr3_ros.yaml.backup")
	if err != nil {
		log.Fatal(err)
	}
	return

}

func add_plugin(path string, backup_path string, new_controller_name string, controller_description string) (err_ error) {
	e := os.Rename(path+"fr3_ros_plugin.xml", path+"fr3_ros_plugin.xml"+".backup")
	if e != nil {
		log.Fatal(e)
	}

	f, err1 := os.Open(path + "fr3_ros_plugin.xml.backup")
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

	err := os.Rename(path+"fr3_ros_plugin.xml.backup", backup_path+"fr3_ros_plugin.xml.backup")
	if err != nil {
		log.Fatal(err)
	}
	return

}
func add_lib_CMAKE(path string, backup_path string, controller_name string) (err_ error) {

	e := os.Rename(path+"CMakeLists.txt", path+"CMakeLists.txt"+".backup")
	if e != nil {
		log.Fatal(e)
	}

	f2, err2 := os.Create(path + "CMakeLists.txt")
	if err2 != nil {
		return err2
	}

	source, err := os.Open(path + "CMakeLists.txt.backup")
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

	err = os.Rename(path+"CMakeLists.txt.backup", backup_path+"CMakeLists.txt.backup")
	if err != nil {
		log.Fatal(err)
	}

	return

}
