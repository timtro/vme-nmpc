import json


class JsonLogParser:

    def __init__(self, fstream):
        self.fstream = fstream

    def getNextObjectAsString(self):
        line = ''
        stringObject = ''
        nestCount = 0
        while '{' not in line:
            line = self.fstream.readline()
            if not line:
                return ''
        stringObject += line
        nestCount += line.count('{')
        nestCount -= line.count('}')
        if nestCount == 0:  # Found single line object
            line = line[0:line.rfind('}') + 1]
            return line
        while True:
            line = self.fstream.readline()
            nestCount += line.count('{')
            nestCount -= line.count('}')
            if nestCount > 0:
                stringObject += line
            else:
                break
        line = line[0:line.rfind('}') + 1]
        stringObject += line
        return stringObject

    def getNextObjectAsDict(self):
        objectString = self.getNextObjectAsString()
        if objectString == "":
            return {}
        else:
            return json.loads(objectString)
