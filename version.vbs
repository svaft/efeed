'===========================================================
'===========================================================

Const sNumberFile = "..\numfile.txt"
Const sBuildFile  = "..\buildno.txt"


'===========================================================
'===========================================================
Function Read_File (Filename)

Dim objFSO
Dim objTextStream
Dim s

Set objFSO = WScript.CreateObject("Scripting.FileSystemObject")
Set objTextStream = objFSO.OpenTextFile(FileName,1,False)

s = Trim (objTextStream.ReadAll)

objTextStream.Close
Set objTextStream = Nothing
Set objFSO = Nothing

Read_File = s

End Function

'===========================================================
'===========================================================
Function Write_File (FileName, Contents)

Dim objFSO
Dim objTextStream
Dim s

Set objFSO = WScript.CreateObject("Scripting.FileSystemObject")
Set objTextStream = objFSO.CreateTextFile (FileName, True)

objTextStream.WriteLine(Contents)

objTextStream.Close
Set objTextStream = Nothing
Set objFSO = Nothing

Write_File = 1

End Function

'===========================================================
'===========================================================

Dim s

s = Read_File (sNumberFile)
s = Int(s) + 1
Write_File sNumberFile, s

s = "#define Build_No " & s & vbCrLf & vbCrLf

Write_File sBuildFile, s


'===========================================================
'===========================================================