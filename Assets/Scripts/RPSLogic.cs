using UnityEngine;

public enum RPSType
{
    R, P, S
}



public class RPSLogic : MonoBehaviour
{
    private RPSType type;
    public RPSType Type {  get { return type; } }

    private RPSView view;

    private void Awake()
    {
        view = GetComponentInChildren<RPSView>();
    }

    public void Start()
    {
        // assign self with random type
        type = (RPSType) Random.Range(0, 3);

        // initial update visual
        view.SetEmoji(type);
    }


    public void LoseTo(RPSLogic winner)
    {
        type = winner.type;
        view.SetEmoji(winner.type);
    }

    //private void OnTriggerEnter2D(Collider2D collision)
    //{

    //    Debug.Log("Collide Trigger");

    //    (RPSLogic winner, RPSLogic loser, bool isTie) = RPSRule.JudgeWinner(this, collision.gameObject.GetComponent<RPSLogic>());

    //    if(!isTie)
    //    {
    //        loser.LoseTo(winner);
    //    }
    //}

    private void Update()
    {
        Collider2D collided = Physics2D.OverlapCircle(transform.position, 0.5f);

        if (collided != null)
        {
            Debug.Log(collided.gameObject.name);
        }
    }

    private void OnCollisionStay2D(Collision2D collision)
    {
        Debug.Log("Collide Collider");

        (RPSLogic winner, RPSLogic loser, bool isTie) = RPSRule.JudgeWinner(this, collision.gameObject.GetComponent<RPSLogic>());

        if (!isTie)
        {
            loser.LoseTo(winner);
        }
    }


}
