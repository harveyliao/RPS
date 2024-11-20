using System;
using UnityEngine;


public class RPSView : MonoBehaviour
{
    [Serializable]
    public struct TypeEmojiPair
    {
        public RPSType type;
        public Sprite sprite;
    }

    //[SerializeField] private Sprite paperEmoji;
    //[SerializeField] private Sprite rockEmoji;
    //[SerializeField] private Sprite scissorsEmoji;

    [SerializeField] private TypeEmojiPair[] typeEmojiPairArr;

    private SpriteRenderer spriteRenderer;

    private void Awake()
    {
        spriteRenderer = GetComponent<SpriteRenderer>();
    }

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame

    public void SetEmoji(RPSType type)
    {

        foreach (TypeEmojiPair pair in typeEmojiPairArr)
        {
            if(pair.type == type)
            {
                spriteRenderer.sprite = pair.sprite;
            }
        }

    }
}
