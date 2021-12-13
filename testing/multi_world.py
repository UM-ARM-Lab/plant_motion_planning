
class Person():

    def __init__(self, name, age):

        self.name = name
        self.age = age

    def print_details(self, all=False):

        print("Name: ", self.name)

        if(all):
            print("Age: ", self.age)


class Student(Person):

    def __init__(self, name, age, school):

        super().__init__(name, age)

        self.school = school

    def print_details(self, all=False):

        print(self.name)
        if(all):
            print(self.school)


# per = Person("abc", 20)
# per.print_details(all=True)

stu = Student("abc", 20, "xyz")
stu.print_details(True)