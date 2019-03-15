sim('PIDControlGenTestDataModel');
dlmwrite('PIDControlTestData.txt', [X.Time, X.Data],'delimiter','\t', 'precision','%.8f');