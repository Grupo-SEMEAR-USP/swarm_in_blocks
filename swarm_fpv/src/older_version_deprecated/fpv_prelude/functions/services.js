//client
var addTwoIntsClient = new ROSLIB.Service({
    ros : ros,
    name : '/add_two_ints',
    serviceType : 'ros_js_tests/multiplier'
  });

var request = new ROSLIB.ServiceRequest({
   a : 2,
   b : 9
});

addTwoIntsClient.callService(request, function(result) {
console.log('Result for service call on '
+ addTwoIntsClient.name
+ ': '
+ result.result);
});


