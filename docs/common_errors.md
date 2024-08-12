### Unity Robotics not found when editing MARUS files

This is due to the packages not in the same assembly as LABUST assembly. To fix this:

1. Open CoreAssembly.asmdef in Assets/marus-core/Scripts
2. Add the following to the references array:
```
"Unity.Robotics.ROSTCPConnector",
"Unity.Robotics.ROSTCPConnector.Messages",
"Unity.Robotics.ROSTCPConnector.MessageGeneration"
```

Save and then navigate back to Unity for the assembly to reload. Voila.