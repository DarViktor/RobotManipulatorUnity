using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;
using UnityEditor.UIElements;



public class MyCustomEditor : EditorWindow
{

    VisualElement container;

    [MenuItem("Test/Test Window")]

    public static void ShowWindow()
    {
        MyCustomEditor window = GetWindow<MyCustomEditor>();
        window.titleContent = new GUIContent(text: "Test Window");
    }

    public void CreateGUI()
    {
        container = rootVisualElement;
        VisualTreeAsset visualTreeAsset = AssetDatabase.LoadAssetAtPath<VisualTreeAsset>("Assets/Assets/MyCustomEditor.uxml");
        container.Add(child: visualTreeAsset.Instantiate());

        StyleSheet styleSheet = AssetDatabase.LoadAssetAtPath<StyleSheet>("Assets/TestStyle.uss");
        container.styleSheets.Add(styleSheet);
    }


}