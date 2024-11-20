using UnityEngine;

public class InstantiationTest : MonoBehaviour
{
    [SerializeField] private int maxAgent;
    [SerializeField] GameObject agentPrefab;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        for (int i = 0; i < maxAgent; i++)
        {
            Vector3 pos = new Vector3(Random.Range(-3f, 3f), Random.Range(-3f, 3f), 0);
            Instantiate(agentPrefab, pos, Quaternion.identity);
        }
    }
}
