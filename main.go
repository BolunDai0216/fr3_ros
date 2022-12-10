package main

// A simple program demonstrating the text input component from the Bubbles
// component library.

import (
	"bufio"
	"fmt"
	"io/ioutil"
	"log"
	"os"
	"strings"

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
	template_file_names    []string

	template_controller textinput.Model
	err                 error
	prog_flow           int
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

	cwd_, err := os.Getwd()
	if err != nil {
		log.Println(err)
	}

	return model{
		textInput:              ti,
		controller_description: ti1,
		custom_cpp_name:        ti3,
		err:                    nil,
		prog_flow:              0,
		curentdir:              cwd_,
		path_CMake:             cwd_ + "/fr3_ros/",
		path_plugin:            cwd_ + "/fr3_ros/",
		path_yaml:              cwd_ + "/fr3_ros/config/",
		path_header:            cwd_ + "/fr3_ros/include/fr3_ros/",
		path_source:            cwd_ + "/fr3_ros/src/",
		path_launch:            cwd_ + "/fr3_ros/launch/",
	}
}

func getClassName(template_controller_name string, path string) (class_name string, err_ error) {
	var temp_arr []string
	var temp string
	fmt.Println(path + template_controller_name + ".h")
	source1, err := os.Open(path + template_controller_name + ".h")
	if err != nil {
		return "s", err
	}

	scanner := bufio.NewScanner(source1) //create scanner for the new file

	for scanner.Scan() {

		if strings.Contains(scanner.Text(), "class") {
			fmt.Println(scanner.Text())

			temp = scanner.Text()
			break
		}

	}

	temp_arr = strings.Fields(temp)
	fmt.Println(temp_arr)

	source1.Close()
	return temp_arr[1], err

}

func populateTemplateChoices(path string) (choices []string) {
	files, err := ioutil.ReadDir(path)
	if err != nil {
		log.Fatal(err)
	}
	for _, files := range files {
		choices = append(choices, strings.Replace(files.Name(), ".cpp", "", -1))
	}
	return choices
}

func populateChoices(user_input_controller_name string) (choices []string) {

	choices = append(choices, user_input_controller_name)
	choices = append(choices, strings.Replace(cases.Title(language.Und, cases.NoLower).String(strings.Replace(user_input_controller_name, "_", " ", -1)), " ", "", -1))
	temp_string_arr := strings.Split(user_input_controller_name, "_")
	temp_string := strings.ToUpper(temp_string_arr[0])
	if len(temp_string) > 1 {
		for i := 1; i < len(temp_string_arr); i++ {
			temp_string += cases.Title(language.Und, cases.NoLower).String(temp_string_arr[i])
		}
	}
	choices = append(choices, temp_string)
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
			switch m.prog_flow {
			case 0: //user chose controller name
				m.prog_flow = 1
				m.cpp_names = populateChoices(m.textInput.Value())
				m.textInput.Blur()
				// return m, cmd
			case 1: //user selected cpp name
				if m.cursor != len(m.cpp_names)-1 {
					m.prog_flow = 3
					m.controller_description.Focus()
				} else {
					m.prog_flow = 2
					m.custom_cpp_name.Focus()
				}
			case 2:
				m.custom_cpp_name.Blur()
				m.controller_description.Focus()
				m.cpp_names[len(m.cpp_names)-1] = m.custom_cpp_name.Value()
				m.prog_flow = 3
			case 3:
				m.template_file_names = populateTemplateChoices(m.path_source)
				m.controller_description.Blur()
				m.prog_flow = 4
			case 4:
				m.prog_flow = 5

			case 5:
				template_controller_code_name, _ := getClassName(m.template_file_names[m.cursor2], m.path_header)
				_ = add_lib_CMAKE(m.path_CMake, m.textInput.Value())
				_ = add_plugin(m.path_plugin, m.textInput.Value(), m.controller_description.Value())
				_ = add_controller_yaml(m.path_yaml, m.template_file_names[m.cursor2], m.textInput.Value())
				_ = create_header(m.path_header, m.template_file_names[m.cursor2], m.textInput.Value(), template_controller_code_name, m.cpp_names[m.cursor])
				_ = create_source(m.path_source, m.template_file_names[m.cursor2], m.textInput.Value(), template_controller_code_name, m.cpp_names[m.cursor])
				_ = create_launch(m.path_launch, m.template_file_names[m.cursor2], m.textInput.Value())
				// fmt.Println(temps)
				m.prog_flow = 6
			case 6:
				return m, tea.Quit

			}
		default:
			switch m.prog_flow {
			case 0:
				m.textInput, cmd = m.textInput.Update(msg)
			case 1:
				switch msg.String() {
				case "up", "k":
					if m.cursor > 0 {
						m.cursor--
					}

				// The "down" and "j" keys move the cursor down
				case "down", "j":
					if m.cursor < len(m.cpp_names)-1 {
						m.cursor++
					}
				}
			case 2:
				m.custom_cpp_name, cmd = m.custom_cpp_name.Update(msg)
			case 3:
				m.controller_description, cmd = m.controller_description.Update(msg)
			case 4:

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
			}

		}

	// We handle errors just like any other message
	case errMsg:
		m.err = msg
		return m, nil
	}
	// switch m.prog_flow {
	// case 0:

	// case 2:
	// 	m.custom_cpp_name, cmd = m.custom_cpp_name.Update(msg)

	// }

	return m, cmd
}

func (m model) View() string {
	// fmt.Println(m.textInput.Value())
	s := ""
	switch m.prog_flow {
	case 6:
		s += "Finished!!!\n"
	case 5:
		s += "You chose your controller name as > \"" + m.textInput.Value() + "\"\n"
		s += "You chose your controller .cpp name as > \"" + m.cpp_names[m.cursor] + ".cpp\"\n"
		s += "Your controller description is> \n   \""
		s += m.controller_description.Value() + "\"\n\n"
		s += "You chose the template as > \"" + m.template_file_names[m.cursor2] + "\"\n\n"
		s += "If all of this information is correct, press enter to generate starter files for your new controller! \nOtherwise press esc to try again"
		s += "\n\n(esc to quit)"
	case 4:
		s += "You chose your controller name as > \"" + m.textInput.Value() + "\"\n"
		s += "You chose your controller .cpp name as > \"" + m.cpp_names[m.cursor] + ".cpp\"\n"
		s += "Your controller description is> \n   \""
		s += m.controller_description.Value() + "\"\n\n"
		s += "Finally, please choose the template controller you'd like to use as a starting point for your new controller>\n\n"
		for i, choice := range m.template_file_names {
			cursor2 := " "
			if m.cursor2 == i {
				cursor2 = ">"
			}
			checked := " "

			s += fmt.Sprintf("%s [%s] %s.cpp\n", cursor2, checked, choice)
		}
		s += "\n\n\n(use up and down arrows to select, enter to enter, esc to quit)\n"

	case 3:

		s += fmt.Sprintf(
			"%s%s%s%s%s",
			"You chose your controller name as > \""+m.textInput.Value()+"\"\n",
			"You chose your controller .cpp name as > \""+m.cpp_names[m.cursor]+".cpp\"\n\n",
			"Almost there! Now please provide a one line description of your new controller\n\n",
			m.controller_description.View(),
			"\n\n(esc to quit)")

	case 2:
		s += fmt.Sprintf("%s%s%s%s",
			"You chose your controller name as > \""+m.textInput.Value()+"\"\n\n",
			"You opted to have a custom name for the cpp class for your controller, please enter it:\n\n",
			m.custom_cpp_name.View(),
			"\n\n(esc to quit)",
		)
	case 1:
		s += "You chose your controller name as > \"" + m.textInput.Value() + "\"\n\n"

		s += "Great! We've generated a few options for your controller class name in the cpp files.\n"
		s += "Choose one of the following name for the cpp class:\n\n"

		for i, choice := range m.cpp_names {
			cursor := " "
			if m.cursor == i {
				cursor = ">"
			}
			checked := " "

			s += fmt.Sprintf("%s [%s] %s.cpp\n", cursor, checked, choice)
		}
		if m.cursor != len(m.cpp_names)-1 {
			s += "\n\n You'd like your cpp name to be " + m.cpp_names[m.cursor] + ".cpp\n"
		} else {
			s += "\n\n Choose this option to enter custom cpp class name for controller\n"
		}
		s += "\n\n\n(use up and down arrows to select, enter to enter, esc to quit)\n"
	default:
		s += fmt.Sprintf(
			"You're about to create a new controller for the FR3 Franka Research Robot\nWhat would you like to name your controller?\n%s\n\n%s",
			m.textInput.View(),
			"(esc to quit)",
		) + "\n"
	}
	return s
}

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
