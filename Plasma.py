import gdb

class Printer:
    def __init__(self, printer, val = None):
        self.val = val
        self.printer = printer
    
    def __call__(self, val):
        return Printer (self.printer, val)
    
    def to_string(self):
        return self.printer(self.val)


def PlStringBuffer(val, decorate = True):
    result = val.type.name + '("' if decorate else '"'
    if val['fSize']:
        len = val['fShort'].type.sizeof / val['fShort'].type.target().sizeof
        if (val['fSize'] < len):
            result += val['fShort'].string()
        else:
            result += val['fData'].referenced_value()['fStringData'].string()
    result += '")' if decorate else '"'
    return result

def PlString(val):
    return 'plString(' + PlStringBuffer(val['fUtf8Buffer'], False) + ')'

    
printers=gdb.printing.RegexpCollectionPrettyPrinter("Plasma")
printers.add_printer('plStringBuffer', '^plStringBuffer<.*>$', Printer(PlStringBuffer))
printers.add_printer('plString', '^plString$', Printer(PlString))
gdb.printing.register_pretty_printer(None, printers, True)
