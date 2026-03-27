# ROS2 Message Notes

## 1. `msg` is for topics, not services

In ROS2, communication types are separated like this:

- `msg`: used for topic publish/subscribe
- `srv`: used for service request/response
- `action`: used for long-running goal-based tasks

So if we make a file like:

```text
more_interfaces/msg/Hw01Numbers.msg
```

that is still for **topic communication only**.

It is **not** a service.

## 2. Why we made `Hw01Numbers.msg`

Your homework goal is:

- publisher reads two integers
- publisher increases `count` every 500 ms
- publisher sends the values
- subscriber computes:
  - `count * int1`
  - `count * int2`

To do that cleanly, the publisher should send the raw data:

- `int1`
- `int2`
- `count`

That is why we made this custom topic message:

```text
int32 int1
int32 int2
int32 count
```

This is better than putting everything into one string, because the subscriber can use each value directly.

## 3. Do I have to make a custom message?

No. You have two choices.

### Choice A: Use `std_msgs/String`

You can send one text string like:

```text
"3 5 7"
```

Then the subscriber must parse the string to recover:

- `int1 = 3`
- `int2 = 5`
- `count = 7`

This works, but it is not a very clean ROS2 design for numeric data.

### Choice B: Use a custom `.msg` file

This is the recommended way when your topic contains structured data.

Advantages:

- each field has a clear meaning
- subscriber code is easier to read
- no string parsing needed
- more similar to real ROS2 package design

## 4. Why use `more_interfaces`?

We used `more_interfaces` because your workspace already had an interface package.

An interface package can contain:

- `msg/`
- `srv/`
- `action/`

So even though `more_interfaces` already had a service file, it can also contain topic message files.

The package name does **not** decide the communication type.
The folder and file type do:

- `msg/*.msg` -> topics
- `srv/*.srv` -> services
- `action/*.action` -> actions

## 5. Can I use only topic protocol?

Yes.

Your clean version is still using only topics:

- publisher publishes `Hw01Numbers`
- subscriber subscribes to `Hw01Numbers`

No service call is involved.

## 6. What changed in our clean version

We added:

- `more_interfaces/msg/Hw01Numbers.msg`
- `hw01_pkg/src/hw01_pub_clean.cpp`
- `hw01_pkg/src/hw01_sub_clean.cpp`

Behavior:

- publisher reads two integers once
- every 500 ms it increases `count`
- publisher sends `int1`, `int2`, and `count`
- subscriber receives them and computes:
  - `count * int1`
  - `count * int2`

## 7. Simple rule to remember

If you want to send multiple related values over a topic:

- quick practice: use `String`
- proper ROS2 way: make a custom `.msg`

For your homework, the custom `.msg` version is the better design.
